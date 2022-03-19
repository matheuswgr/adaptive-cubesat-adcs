#ifndef __iterator_h
#define __iterator_h

template<typename T>
class Iterator
{
    public:
        virtual T* next(){return new T;};
        virtual bool hasNext(){return false;};
};

#endif