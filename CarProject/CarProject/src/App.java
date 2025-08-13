public class App {
    public static void main(String[] args) throws Exception {
        //random print line :)
        System.out.println("1405\n");
        
        //creating cars
        Car myCar = new Car();
        Car lilysCar = new Car();
        Car bensCar = new Car();
        Car conradsCar = new Car();
        Car zacsCar = new Car();

        //setting models
        myCar.setModel("Thrifty Mobile");
        lilysCar.setModel("Cool Teal Jeep");
        bensCar.setModel("2013 Toyota Corrola");
        conradsCar.setModel("Ancient Ferrari");
        zacsCar.setModel("Brand New Tesla CyberTruck");


        myCar.accelerate(3.2);
        lilysCar.accelerate(5);

        //printing speed and models
        System.out.println("My car's speed: " + myCar.getSpeed() + "\nMy car's model: " + myCar.getModel());

        System.out.println("\nLily's car's speed: " + lilysCar.getSpeed() + "\nLily's car model: " + lilysCar.getModel());

        System.out.println("\nMy car's speed: " + myCar.getSpeed() + "\nMy car model: " + myCar.getModel());

        System.out.println("\nBen's car's speed: " + bensCar.getSpeed() + "\nBen's car model: " + bensCar.getModel());

        System.out.println("\nConrad's car's speed: "+ conradsCar.getSpeed() + "\nConrad's car model: " + conradsCar.getModel());

        System.out.println("\nZac's car's speed: " + zacsCar.getSpeed() + "\nZac's car model: " + zacsCar.getModel());

        //creating new drivers
        Driver macy = new Driver();
        Driver lily = new Driver();
        Driver ben = new Driver();
        Driver conrad = new Driver();
        Driver zac = new Driver();
        //System.out.println("\n\n\nCurrent temperature is: " + zac.driverPreferedCarTemperature);
       
        //prefered temperatures
        macy.driverPreferedCarTemperature = 63;
        lily.driverPreferedCarTemperature = 69;
        ben.driverPreferedCarTemperature = 65;
        conrad.driverPreferedCarTemperature = 70;
        zac.driverPreferedCarTemperature = 68;

        int randomTemp = 9;

        //TODO: Fix this
        //picking random temp
        myCar.setTemperature(randomTemp);
        lilysCar.setTemperature(randomTemp);
        bensCar.setTemperature(randomTemp);
        conradsCar.setTemperature(randomTemp);
        zacsCar.setTemperature(randomTemp);

        //ac will go here

        //TODO: Fix this
        //adjusting temperatures in cars
        myCar.setTemperature(macy.driverPreferedCarTemperature);
        lilysCar.setTemperature(lily.driverPreferedCarTemperature);
        bensCar.setTemperature(ben.driverPreferedCarTemperature);
        conradsCar.setTemperature(conrad.driverPreferedCarTemperature);
        zacsCar.setTemperature(zac.driverPreferedCarTemperature);

        //message about ac on
        //TODO: ask Steven if this would this be more efficent as a boolean?
        //TODO: Fix this
            if(randomTemp > driverPreferedCarTemperature){
            System.out.println("AC is on. Car is cooling down. (wind sound effects)");
        }
         else if(randomTemp < driverPreferedCarTemperature){
            System.out.println("Heating is turned on. Car will be warm shortly. (Car heats up)");
        }
            else{
            System.out.println("Car is at the driver's prefered car temperature. If you are in the passenger seat, too bad for you.");
            System.out.println("If you would like to change the driver's prefered car temperature, please stop driving and park somewhere. Driving and programing java are two difficult things and they should not be attempted at the same time. Do not multitask while driving.\n Have a pleasant day at YOUR prefered temperature.");
        }


        System.out.println("Current temperature is: " + myCar.getTemperature() + "\nMy friend's car's temperature: " + lilysCar.getTemperature());
        
       /*  myCar.myRadio.getStation();
        System.out.println(myCar.myRadio.getStation());*/

       String currentGenre = GenreHandler.StationToGenre("100.3");
        System.out.println(currentGenre);


    }
}