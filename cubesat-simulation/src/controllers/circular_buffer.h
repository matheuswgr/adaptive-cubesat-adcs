#ifndef __circular_buffer_h
#define __circular_buffer_h

#include "iterator.h"


template<typename T>
class CircularBuffer
{   
    template<typename> friend class CircularBufferIterator;
    private:
        T* buffer;
        int head; 
        int tail;
        bool full;

    public:
        int size;

    public:
        CircularBuffer(int bufferSize)
        {
            buffer = new T[bufferSize]();
            head = 0;
            tail = 0;
            size = bufferSize;
            full = false;
        }

        void push(T value)
        {
            buffer[head] = value;

            if(full)
            {
                tail = (tail + 1) % size;
            }

            head = (head + 1) % size;

            full = (head == tail);
        }  
                
        Iterator<T>* getIterator();
};

template<typename T>
class CircularBufferIterator : public Iterator<T>
{
    private:
        CircularBuffer<T>* buffer;
        int nextIndex; 
        int visted = 0;

    public:
        CircularBufferIterator(CircularBuffer<T>* buffer)
        {  
            this->buffer = buffer;
            this->nextIndex =buffer->head;
        }

        T* next()
        {
            if (nextIndex == 0)
            {
                nextIndex = buffer->size-1;
            }
            else
            {
                nextIndex--;
            }
            visted++;
            return &this->buffer->buffer[nextIndex];
        }
        
        bool hasNext()
        {
            if (visted == (buffer->size))
            {
                return false;
            }
            else
            {
                return true;
            }
        }
};

template<typename T>
Iterator<T>* CircularBuffer<T>::getIterator()
{
  return new CircularBufferIterator<T>(this);
}


#endif