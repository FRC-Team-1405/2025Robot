import java.util.HashMap;
import java.util.List;
import java.util.Random;

public class App {
    public static void main(String[] args) throws Exception {
        System.out.println("on your mark get set Goooo!!!!!!");
        Car myCar = new Car();
        Random random = new Random();
        int randomInt = random.ints(0, 100).findFirst().getAsInt();
        myCar.exelorate (randomInt);
    
        Car myCar2 = new Car();
        Random random2 = new Random();
        int randomInt2 = random2.ints(0, 100).findFirst().getAsInt();
        myCar2.exelorate (randomInt2);

       //HashMap<Car, int> CarSpeedSorter;


        System.out.println("Car1 speed: " + randomInt);

        System.out.println("Car2 speed: " + randomInt2);

        if (myCar2.Crash()) { System.out.println("Car2 Crashed in an fiery inferno");
        
        }

        if (myCar.Crash()) { System.out.println("Car1 Crashed in an fiery inferno"); 

        }

        if (myCar.getspeed() < myCar2.getspeed()) {System.out.println("Car2 Wins!!!!!");
            
        }
        if (myCar.getspeed() > myCar2.getspeed()) {System.out.println("Car1 Wins!!!!!");
            
        }
    }
}
