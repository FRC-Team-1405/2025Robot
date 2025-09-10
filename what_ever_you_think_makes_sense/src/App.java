import java.util.HashMap;
import java.util.Random;

public class App {

    public static void main(String[] args) throws Exception {
     
     HashMap  <Car, double > 
     .put myCar , randomint;
     .put myCar2, randomint2;
     .put myCar3, randomint3;
     
        Random random = new Random();
        double randomint = random.nextDouble(100,150);   //random number generator
        double randomint2 = random .nextDouble(100,150); 
        double randomint3 = random. nextDouble(100,150);
       // double randomint4 = random 

        Car myCar = new Car();                       //car 1 speed
        myCar.exelorate(randomint);
        System.out.println( myCar. getspeed());
        

        Car myCar2 = new Car();                      //car 2 speed
        myCar2.exelorate(randomint2);
        System.out.println(myCar2.getspeed()); 


        Car myCar3 = new Car();                      //car 3 speed
        myCar3.exelorate(randomint3);
        System.out.println(myCar3.getspeed()); 


        


        if (myCar.getspeed()> myCar2.getspeed()&& myCar2.getspeed()>myCar3.getspeed()){  
            System.out.println("FORD - HONDA - KIA");
        }
        if (myCar.getspeed()> myCar3.getspeed()&& myCar3.getspeed()>myCar2.getspeed()){    
            System.out.println("FORD - KIA - HONDA");
        }
        if (myCar2.getspeed()> myCar.getspeed()&& myCar.getspeed()>myCar3.getspeed()){    
            System.out.println("HONDA - FORD - KIA");
        }
        if (myCar2.getspeed()> myCar3.getspeed()&& myCar3.getspeed()>myCar.getspeed()){    
            System.out.println("HONDA - KIA - FORD");
        }
        if (myCar3.getspeed()> myCar2.getspeed()&& myCar2.getspeed()>myCar.getspeed()){   
            System.out.println("KIA - HONDA - FORD");
        }
        if (myCar3.getspeed()> myCar.getspeed()&& myCar.getspeed()>myCar2.getspeed()){    
            System.out.println("KIA - FORD - HONDA");
        }


        if ( myCar2.getspeed() > 145){    
            System.out.println(" HONDA False Start");
        }
        if ( myCar3.getspeed() > 145){    
            System.out.println(" KIA False Start");
        }
        if ( myCar.getspeed() > 145){    
            System.out.println(" FORD False Start");
        }
        }

    }