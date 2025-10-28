#pragma once

#include <vector>
#include <iostream>
#include <algorithm>


template <class T>
class CircularBuffer{
public:

	CircularBuffer(uint max_size) :
        max_size_(max_size),
        values(std::vector<T>(max_size,0))
    {}

	CircularBuffer(uint max_size, T init) :
        max_size_(max_size),
        values(std::vector<T>(max_size,init))
    {}

    // T operator[](int) = delete;

    void fill(T value){
        std::fill(values.begin(), values.end(), value);
    }

	void put(T item){
        values[ctr%max_size_] = item;
        ctr++;
        if (ctr==max_size_+1)
        full = true;
    }

    uint count(T key){
        return std::count_if(values.begin(), values.end(), [key](T entry) { return entry == key; });
    }

    T get(unsigned int i){
        i = (ctr-max_size_ + i)%max_size_;
        // std::cout<<i<<std::endl;
        return values[i];
    }

    std::size_t size(){
        if (full)
            return max_size_;
        return ctr+1;
    }

    std::size_t max_size(){
        return max_size_;
    }

    bool isfull(){
        return full;
    }

    T mean(){
        // std::cout<<"----\n";
        T sum = 0;
        for (uint i = 0; i<size(); i++){
            sum += get(i);
            // std::cout<<get(i)<<std::endl;
        }
        T mean = sum/(T)size();
        // std::cout<<"mean: "<<mean<<"\n";
        return mean; 
    }

private:
    unsigned int ctr = 0;
    std::size_t size_ = 0;
    const std::size_t max_size_ = 0;
    bool full = false;
    std::vector<T> values; 
};