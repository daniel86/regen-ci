#ifndef REGEN_STACK_H_
#define REGEN_STACK_H_

namespace regen {
	/**
	 * \brief A simple stack implementation.
	 */
	template<typename T>
	class Stack {
	public:
		/**
		 * \brief A Node in the stack.
		 */
		class Node {
		public:
			/**
			 * @param value node value
			 * @param next next node or nullptr
			 */
			Node(const T &value, Node *next)
					: value_(value), next_(next) {}

			/** node value. */
			T value_;
			/** next node or nullptr. */
			Node *next_;
		};

		Stack() : top_(nullptr) {}

		~Stack() {
			while (top_ != nullptr) {
				Node *buf = top_;
				top_ = top_->next_;
				delete buf;
			}
		}

		/**
		 * Sets top value.
		 */
		void set_value(const T &value) {
			top_->value_ = value;
		}

		/**
		 * Push value onto the stack.
		 */
		void push(const T &value) {
			top_ = new Node(value, top_);
		}

		/**
		 * Push value to the stack bottom.
		 */
		void pushBottom(const T &value) {
			Node *root = bottomNode();
			if (root == nullptr) {
				top_ = new Node(value, nullptr);
			} else {
				root->next_ = new Node(value, nullptr);
			}
		}

		/**
		 * Pop the top value.
		 */
		void pop() {
			if (top_ == nullptr) { return; }
			Node *buf = top_;
			top_ = top_->next_;
			delete buf;
		}

		/**
		 * Pop the top value. But do not delete the node.
		 * You have to free the top node yourself after calling this.
		 */
		void popKeepNode() {
			if (top_ != nullptr) { top_ = top_->next_; }
		}

		/**
		 * Pops the bottom value.
		 */
		void popBottom() {
			if (!top_ || !top_->next_) {
				// empty or single element
				pop();
				return;
			}
			for (Node *n = top_; n != nullptr; n = n->next_) {
				Node *root = n->next_;
				if (!root->next_) {
					n->next_ = nullptr;
					delete root;
					break;
				}
			}
		}

		/**
		 * @return top value.
		 */
		const T &top() const { return top_->value_; }

		/**
		 * You can use the top node to iterate through the stack.
		 * @return top value node.
		 */
		Node *topNode() { return top_; }

		/**
		 * @return bottom value node.
		 */
		Node *bottomNode() {
			if (!top_) return nullptr;
			Node *root = top_;
			for (; root->next_ != nullptr; root = root->next_) {}
			return root;
		}

		/**
		 * @return if the stack is empty.
		 */
		bool isEmpty() const { return top_ == nullptr; }

	private:
		Node *top_;
	};
} // namespace

#endif /* REGEN_STACK_H_ */
