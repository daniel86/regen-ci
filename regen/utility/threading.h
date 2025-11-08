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

	// i have a strange problem with boost::this_thread here.
	// it just adds 100ms to the interval provided :/
#ifdef UNIX
#define usleepRegen(v) usleep(v)
#else
#define usleepRegen(v) boost::this_thread::sleep(boost::posix_time::microseconds(v))
#endif
} // namespace

#endif /* REGEN_THREADING_H_ */
