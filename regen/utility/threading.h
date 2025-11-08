#ifndef REGEN_THREADING_H_
#define REGEN_THREADING_H_

#include <boost/thread/thread.hpp>
#include <thread>
#include <atomic>
#include <vector>
#include <functional>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386) || defined(_M_IX86)
    #include <immintrin.h>
    #define CPU_PAUSE() _mm_pause()
#elif defined(__aarch64__) || defined(__arm__)
    #define CPU_PAUSE() asm volatile("yield" ::: "memory")
#else
    #define CPU_PAUSE() ((void)0)
#endif

namespace regen {
	/**
	 * \brief A simple spin lock implementation.
	 * This is a low-level lock that can be used for short critical sections.
	 * It is not suitable for long operations as it can lead to high CPU usage.
	 */
	class SpinLock {
		std::atomic_flag flag = ATOMIC_FLAG_INIT;
	public:
		void lock() {
			while (flag.test_and_set(std::memory_order_acquire)) {
				CPU_PAUSE();
			}
		}

		void unlock() {
			flag.clear(std::memory_order_release);
		}
	};

	/**
	 * \brief An adaptive lock that spins for a while before blocking.
	 * This is useful for situations where contention is expected to be low.
	 */
	class AdaptiveLock {
		std::atomic_flag flag = ATOMIC_FLAG_INIT;
	public:
		void lock() {
			int spins = 0;
			while (flag.test_and_set(std::memory_order_acquire)) {
				if (++spins > 1000)
					std::this_thread::sleep_for(std::chrono::nanoseconds(50));
			}
		}
		void unlock() {
			flag.clear(std::memory_order_release);
		}
	};

	/**
	 * \brief A simple thread pool that uses spin-waiting.
	 * This is a simple thread pool implementation that uses spin-waiting
	 * for task assignment.
	 */
	class SpinThreadPool {
	public:
		/**
		 * \brief A worker thread in the thread pool.
		 */
		struct Worker {
			std::thread thread;
			std::atomic<bool> has_work{false};
			std::function<void()> job;
		};

		/**
		 * \brief Constructor for the thread pool.
		 * @param numThreads the number of threads in the pool.
		 */
		explicit SpinThreadPool(unsigned int numThreads);

		unsigned int numThreads() const { return workers.size(); }

		/**
		 * Assign a job to a worker thread.
		 * @param i the index of the worker thread.
		 * @param f the job to be executed.
		 */
		void assignJob(int i, std::function<void()> f);

		/**
		 * Wait for all worker threads to finish their jobs.
		 */
		void waitAll();

		/**
		 * Shutdown the thread pool.
		 */
		void shutdownPool();

	private:
		std::vector<Worker> workers;
		std::atomic<bool> shutdown;
	};

	/**
	 * \brief A job structure representing a unit of work.
	 */
	struct Job {
		void (*fn)(void*) = nullptr;
		void *arg = nullptr;
	};

	/**
	 * \brief A lock-free job queue using a circular buffer.
	 */
	class JobQueue {
		std::vector<Job> buffer;
		// Producer position, padded to avoid false sharing
		alignas(64) std::atomic<uint32_t> head{0};
		// Consumer position, padded to avoid false sharing
		alignas(64) std::atomic<uint32_t> tail{0};
	public:
		JobQueue() : buffer(1024) {
			assert((buffer.size() & (buffer.size() - 1)) == 0 && "JobQueue size must be power of two");
		}

		/**
		 * Grow the job queue to double its current size.
		 * This is not thread-safe and should only be called by the owner thread
		 * when no frame is active.
		 */
		void grow() {
			size_t oldSize = buffer.size();
			size_t newSize = oldSize * 2;
			std::vector<Job> newBuffer(newSize);
			uint32_t t = tail.load(std::memory_order_acquire);
			uint32_t h = head.load(std::memory_order_acquire);
			uint32_t count = (h - t) & (oldSize - 1);
			for (uint32_t i = 0; i < count; ++i) {
				newBuffer[i] = buffer[(t + i) & (oldSize - 1)];
			}
			head.store(count, std::memory_order_release);
			tail.store(0, std::memory_order_release);
			buffer = std::move(newBuffer);
		}

		/**
		 * Producer non-blocking
		 * @param j The job to push.
		 * @return True if the job was pushed, false if the queue is full.
		 */
		bool push(const Job &j) {
			uint32_t h = head.load(std::memory_order_relaxed);
			uint32_t t = tail.load(std::memory_order_acquire);
			if ((h - t) == buffer.size() - 1) {
				return false; // full
			}
			buffer[h & (buffer.size() - 1)] = j;
			head.store(h+1, std::memory_order_release);
			return true;
		}

		/**
		 * Consumer non-blocking
		 * @param out The job to pop.
		 * @return True if a job was popped, false if the queue is empty.
		 */
		bool try_pop(Job& out) {
			uint32_t t = tail.load(std::memory_order_relaxed);
			uint32_t h = head.load(std::memory_order_acquire);
			if (t == h) {
				return false; // empty
			}
			out = buffer[t & (buffer.size() - 1)];
			tail.store(t+1, std::memory_order_release);
			return true;
		}

		/**
		 * Checks if there are unassigned jobs in the job queue.
		 * @return True if there are unassigned jobs, false otherwise.
		 */
		bool hasUnassignedJobs() const {
			uint32_t t = tail.load(std::memory_order_acquire);
			uint32_t h = head.load(std::memory_order_acquire);
			return (t & (buffer.size() - 1)) != (h & (buffer.size() - 1));
		}
	};

	/**
	 * \brief A job pool for managing and executing jobs across multiple threads.
	 */
	class JobPool {
	public:
		/**
		 * \brief Constructor for the job pool.
		 * @param numThreads The number of worker threads in the pool.
		 */
		explicit JobPool(uint32_t numThreads) : numThreads_(numThreads) {
			threads_.reserve(numThreads_);
			for (uint32_t i = 0; i < numThreads_; ++i) {
				threads_.emplace_back(&JobPool::workerLoop, this);
			}
		}

		/**
		 * \brief Destructor for the job pool.
		 * Stops all worker threads and cleans up resources.
		 */
		~JobPool() {
			stopThreads();
		}

		/**
		 * Stops all worker threads in the pool.
		 */
		void stopThreads() {
			running_.store(false);
			workSem_.release(static_cast<int>(numThreads_));
			for (auto &thread : threads_) {
				if (thread.joinable()) {
					thread.join();
				}
			}
		}

		/**
		 * Adds a job to the job queue before beginFrame() was called
		 * to fill the job queue for each frame.
		 * @param job The job to add.
		 */
		void addJobPreFrame(const Job &job) {
			if (jobQueue_.push(job)) {
				numPushed_++;
			} else {
				// job queue full, grow and push
				jobQueue_.grow();
				jobQueue_.push(job);
				numPushed_++;
			}
		}

		/**
		 * Adds a job to the job queue within a frame.
		 * @param job The job to add.
		 */
		void addJobWithinFrame(const Job &job) {
			if (jobQueue_.push(job)) {
				numJobsRemaining_.fetch_add(1u, std::memory_order_acq_rel);
				workSem_.release(1);
			} else {
				// job queue full, execute directly
				job.fn(job.arg);
			}
		}

		/**
		 * Begins a new frame for job processing.
		 * This should be called by the owner thread after all jobs for the frame have been added.
		 * This is a non-blocking call, endFrame() must be called to ensure all jobs are completed.
		 */
		void beginFrame() {
			numJobsRemaining_.store(numPushed_, std::memory_order_relaxed);
			workSem_.release(numPushed_);
		}

		/**
		 * Performs a job from the job queue.
		 * This can be called by worker threads to execute a job.
		 * @param job The job to perform.
		 */
		void performJob(const Job &job) {
			job.fn(job.arg);
			numJobsRemaining_.fetch_sub(1u, std::memory_order_acq_rel);
		}

		/**
		 * Checks if there are unassigned jobs in the job queue.
		 * @return True if there are unassigned jobs, false otherwise.
		 */
		bool hasUnassignedJobs() const {
			return jobQueue_.hasUnassignedJobs();
		}

		/**
		 * Steals an unassigned job from the job queue.
		 * This can be called by the owner thread or worker threads to get a job to execute.
		 * @param out The job to steal.
		 * @return True if a job was stolen, false otherwise.
		 */
		bool stealJob(Job &out) {
			return jobQueue_.try_pop(out);
		}

		/**
		 * Ends the current frame for job processing.
		 * This should be called by the owner thread to wait for all jobs to complete.
		 * This is a blocking call.
		 */
		void endFrame() {
#if 0
			int spins = 0;
			while (numJobsRemaining_.load(std::memory_order_acquire) > 0u) {
				if (++spins < 1000) _mm_pause();
				else std::this_thread::yield();
			}
#else
			while (numJobsRemaining_.load(std::memory_order_acquire) > 0u) {
				CPU_PAUSE();
			}
#endif
			numPushed_ = 0u;
		}

		/**
		 * Worker thread loop for processing jobs.
		 * This should be run by each worker thread in the pool.
		 */
		void workerLoop() {
			Job job;
			while (running_.load()) {
				workSem_.acquire();
				if (jobQueue_.try_pop(job)) {
					performJob(job);
				}
			}
		}

	protected:
		// number of worker threads
		const uint32_t numThreads_;
		// worker threads
		std::vector<std::thread> threads_;

		// job queue
		JobQueue jobQueue_;
		// semaphore to signal available work
		std::counting_semaphore<> workSem_{0};
		// number of jobs remaining in the current frame
		std::atomic<uint32_t> numJobsRemaining_{0u};
		// running flag
		std::atomic<bool> running_{true};
		// number of jobs pushed in the current frame
		uint32_t numPushed_ = 0u;
	};

	// i have a strange problem with boost::this_thread here.
	// it just adds 100ms to the interval provided :/
#ifdef UNIX
#define usleepRegen(v) usleep(v)
#else
#define usleepRegen(v) boost::this_thread::sleep(boost::posix_time::microseconds(v))
#endif
} // namespace

#endif /* REGEN_THREADING_H_ */
