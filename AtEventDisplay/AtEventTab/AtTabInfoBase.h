#ifndef ATTABINFOBASE_H
#define ATTABINFOBASE_H

class AtTabInfoBase {
public:
   AtTabInfoBase() = default;
   virtual ~AtTabInfoBase() = default;

   /**
    * @brief setup how to access this info from its data source.
    */
   virtual void Init() = 0;
   /**
    * @brief update the data this holds from its source.
    */
   virtual void Update() = 0;
};

#endif
