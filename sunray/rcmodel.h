// Ardumower Sunray 

// R/C model control

#ifndef RCMODEL_H
#define RCMODEL_H



class RCModel {
    public:
      void begin();      
      void run();      
    protected:
      float lin_PPM ;                                            
      float linearPPM ;                                         
      float ang_PPM ;                                            
      float angularPPM ;                                         
      int buttontimer ;                                          
      bool RC_Mode ; 
      unsigned long nextControlTime ;

};


#endif

