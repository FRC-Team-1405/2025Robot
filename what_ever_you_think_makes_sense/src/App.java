import java.util.Random;

public class App {
    public static void main(String[] args) throws Exception {
        Random random = new Random();
        double randomDouble = random.nextDouble();  //random number generator
        double randomDouble2 = random.nextDouble(); 
        double randomDouble3 = random.nextDouble(); 


        Car myCar = new Car();                       //car 1 speed
        myCar.exelorate(randomDouble);
        System.out.println( myCar. getspeed());
        

        Car myCar2 = new Car();                      //car 2 speed
        myCar2.exelorate(randomDouble2);
        System.out.println(myCar2.getspeed()); 


        Car myCar3 = new Car();                      //car 3 speed
        myCar3.exelorate(randomDouble3);
        System.out.println(myCar3.getspeed()); 


        


        if (myCar.getspeed()> myCar2.getspeed()&& myCar2.getspeed()>myCar3.getspeed()){  
            System.out.println("1 - 2 - 3");
        }
        if (myCar.getspeed()> myCar3.getspeed()&& myCar3.getspeed()>myCar2.getspeed()){    
            System.out.println("1 - 3 - 2");
        }
        if (myCar2.getspeed()> myCar.getspeed()&& myCar.getspeed()>myCar3.getspeed()){    
            System.out.println("2 - 1 - 3");
        }
        if (myCar2.getspeed()> myCar3.getspeed()&& myCar3.getspeed()>myCar.getspeed()){    
            System.out.println("2 - 3 - 1");
        }
        if (myCar3.getspeed()> myCar2.getspeed()&& myCar2.getspeed()>myCar.getspeed()){   
            System.out.println("3 - 2 - 1");
        }
        if (myCar3.getspeed()> myCar.getspeed()&& myCar.getspeed()>myCar2.getspeed()){    
            System.out.println("3 - 1 - 2");
        }
       

        }

    }