#ifndef __smartdata_h
#define __smartdata_h

#include <list>
#include <string>

class Coordinates
{
    public:
        float x;
        float y;
        float z;
    
    public:
        Coordinates(float x, float y, float z)
        {
            this->x = x;
            this->y = y;
            this->z = z;
        }
};

class Region
{
    public:
        float x;
        float y;
        float z;
        float radius;
        unsigned long initialTime;
        unsigned long finalTime;
    public:
        Region(float x, float y, float z, float radius, unsigned long initialTime, unsigned long finalTime)
        {
            this->x = x;
            this->y = y;
            this->z = z;
            this->radius = radius;
            this->initialTime = initialTime;
            this->finalTime = finalTime;
        }
};

class SmartData
{  
    template<typename Transformer, typename ValueList>
    friend class TransformerSmartData; 

    protected:
        Coordinates* locationCoordinates;
        unsigned int period;
        unsigned long currentTime;
    
    public:
        float smartdataValue;

    public:
        //virtual Value value() = 0;
        //virtual Value update() = 0;
        virtual void wait() =0;
        
        Coordinates location()
        {
            return *locationCoordinates;
        }

        unsigned long time()
        {
            return currentTime;
        }
};

// Initially  just a ros publisher, subsriber or simply transformer node.
// Everything is time triggered
/*template<typename Transducer, typename Value, typename Message>
class SmartData :  public rclcpp::Node
{
    friend Transducer;    

    private:
        static const unsigned long UNIT = Transducer::UNIT;
        static const bool active = Transducer::active;

        Value _value;
        Transducer* transducer;
        
        int device;
        
        unsigned long expiry;
        
        unsigned long time;
        
        int mode;

        Coordinates* location;
        Region* region;

        rclcpp::TimerBase::SharedPtr refreshTimer;

        list<SmartData>* smartdataList;
        
    public:
        // expiry and period are given in milliseconds
        SmartData(unsigned int dev, int expiry, int mode, int period, string nodeName, Coordinates* location) : Node(nodeName)
        {
            transducer = new Transducer();
            _value = transducer->sense();

            mode = mode;
            device = dev;
            expiry = expiry;
            refreshPeriod = period;
            location = location;
            
            refreshTimer = this->create_wall_timer(std::chrono::milliseconds(refreshPeriod), std::bind(&ResponsiveSmartData<Transducer, Value, Message>::publish, this));
        }

        SmartData(Region* region, int expiry, int period, string nodeName, Coordinates* location) : Node(nodeName)
        {
            region = region;
            expiry = expiry;
            period = period;
            location = location;

            // derive subscriber from region
            // derive publisher from update
        }

        SmartData(int period, list<SmartData>* smartdataList)
        {
            period = period;
            smartdataList = smartdataList;
            
            refreshTimer = this->create_wall_timer(std::chrono::milliseconds(refreshPeriod), std::bind(&ResponsiveSmartData<Transducer, Value, Message>::publish, this));
        }

        Value value()
        {
            return _value;
        }

        void update(Value _value)
        {
            this->_value = _value;
        }

        Coordinates location()
        {
            return this->location;
        }

        unsigned long time()
        {
            return this->time;
        }

    private:
        void publish()
        {

        }
};*/

#endif
