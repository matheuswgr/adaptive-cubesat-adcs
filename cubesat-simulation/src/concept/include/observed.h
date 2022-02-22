#ifndef __observed_h
#define __observed_h

#include <list>
#include <iostream>

template<typename T>
class Observed
{
    private:
        std::list<T*> observers;
        int id = -1;

    public:
        void startObserving(T* observer)
        {
            observers.push_front(observer);
        }

        void notifyObservers()
        {
            for (typename std::list<T*>::iterator it = this->observers.begin(); it != this->observers.end(); ++it)
            {
                (*it)->notify();
            }           
        }

        void giveId(int id)
        {
            this->id = id;
        }

        void notifyObserversWithId()
        {
            for (typename std::list<T*>::iterator it = this->observers.begin(); it != this->observers.end(); ++it)
            {
                (*it)->notify(this->id);
            }           
        }
};

#endif