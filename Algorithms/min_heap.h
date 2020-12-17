//
// Created by r00t on 12/17/20.
//

#ifndef AI_PROJECT_MIN_HEAP_H
#define AI_PROJECT_MIN_HEAP_H

#include <vector>
#include <algorithm>
template <class T>
class min_heap {
private:
    std::vector<T> _vec;
public:
    min_heap() = default;

    void push_back(T& val);

    void pop();

    T& top();

    bool empty();

    void erase(typename std::vector<T>::const_iterator);

    typename std::vector<T>::const_iterator begin() const;

    typename std::vector<T>::const_iterator end() const;


};

template<class T>
void min_heap<T>::push_back(T &val) {
    _vec.push_back(val);
}

template<class T>
T &min_heap<T>::top() {
    std::make_heap(_vec.begin(),_vec.end());
    return _vec.back();
}

template<class T>
void min_heap<T>::pop() {
    _vec.pop_back();

}

template<class T>
bool min_heap<T>::empty() {
    return _vec.empty();
}

template<class T>
typename std::vector<T>::const_iterator min_heap<T>::begin() const {
    return _vec.begin();
}

template<class T>
typename std::vector<T>::const_iterator min_heap<T>::end() const {
    return _vec.end();
}

template<class T>
void min_heap<T>::erase(typename std::vector<T>::const_iterator it){
    _vec.erase(it);
}


#endif //AI_PROJECT_MIN_HEAP_H
