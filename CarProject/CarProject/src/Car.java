import java.util.Random;

public class Car {
    private Driver driver;
    private double speed = 0;

    private String model = "Ford";

    private int carTemperature = 0;


    public Car(Driver driver){
        this.driver = driver; 

        int setRandomTemp = new Random().nextInt(100);
        carTemperature = setRandomTemp;
    }


    public void accelerate(double pressure){
        speed = pressure*10;
    }

    public double getSpeed(){
        return speed;
    }

    public void setModel(String Model){
        model = Model;
    }
    public String getModel(){
        return model;
    }
    

    public void setTemperature(Driver myDriver){
        carTemperature = myDriver.driverPreferedCarTemperature;
    }

    public double getTemperature(){
        return carTemperature;
    }

    public void acAndHeating(){
             if(carTemperature > driver.driverPreferedCarTemperature){
                System.out.println("\nAC is on. Car is cooling down. (wind sound effects)");
                while(carTemperature > driver.driverPreferedCarTemperature){
                    carTemperature -= 1;
                    System.out.println("Current Temperature: " + carTemperature);
            
                }
               
                System.out.println("\nCar is at the driver's prefered car temperature.");
                System.out.println("\n\nIf you would like to change the driver's prefered car temperature, please stop driving and park somewhere. \n\nDriving and programing java are two difficult things and they should not be attempted at the same time. \n\nDo not multitask while driving.\n\nHave a pleasant day at YOUR prefered temperature.");
    
        }
            else if(carTemperature < driver.driverPreferedCarTemperature){
            System.out.println("\nHeating is turned on. Car will be warm shortly. (Car heats up)");
            while(carTemperature < driver.driverPreferedCarTemperature){
                carTemperature += 1;
                System.out.println("Current Temperature: " + carTemperature);
            }
           
            System.out.println("\nCar is at the driver's prefered car temperature.");
            System.out.println("\n\nIf you would like to change the driver's prefered car temperature, please stop driving and park somewhere. \n\nDriving and programing java are two difficult things and they should not be attempted at the same time. \n\nDo not multitask while driving.\n\nHave a pleasant day at YOUR prefered temperature.");

            
        }
            else{
            System.out.println("\nCar is at the driver's prefered car temperature.");
            System.out.println("\n\nIf you would like to change the driver's prefered car temperature, please stop driving and park somewhere. \n\nDriving and programing java are two difficult things and they should not be attempted at the same time. \n\nDo not multitask while driving.\n\nHave a pleasant day at YOUR prefered temperature.");
        }
    }

    public Radio myRadio = new Radio();
}