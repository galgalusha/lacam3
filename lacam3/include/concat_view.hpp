#pragma once

template <typename T>
class ConcatView {
 public:
  ConcatView(T self, const std::vector<T>& rest)
      : self_(self), rest_(rest) {}

  class Iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = const T*;
    using reference = T;

    Iterator(T self, typename std::vector<T>::const_iterator it, bool at_self)
        : self_(self), it_(it), at_self_(at_self) {}

    reference operator*() const { return at_self_ ? self_ : *it_; }

    Iterator& operator++() {
      if (at_self_) {
        at_self_ = false;
      } else {
        ++it_;
      }
      return *this;
    }

    bool operator!=(const Iterator& other) const {
      return at_self_ != other.at_self_ || it_ != other.it_;
    }

   private:
    T self_;
    typename std::vector<T>::const_iterator it_;
    bool at_self_;
  };

  Iterator begin() const { return Iterator(self_, rest_.cbegin(), true); }
  Iterator end() const { return Iterator(self_, rest_.cend(), false); }
  size_t size() const { return rest_.size() + 1; }

 private:
  T self_;
  const std::vector<T>& rest_;
};

