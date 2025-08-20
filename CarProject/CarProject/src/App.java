public class App {
    public static void main(String[] args) throws Exception {
        //random print line :)
        System.out.println("Welcome to CarMart! We are delighted to serve you.\n");
        
        //creating new drivers
        Driver macy = new Driver();
        Driver lily = new Driver();
        Driver ben = new Driver();
        Driver conrad = new Driver();
        Driver zac = new Driver();

        //creating cars
        Car myCar = new Car(macy);
        Car lilysCar = new Car(lily);
        Car bensCar = new Car(ben);
        Car conradsCar = new Car(conrad);
        Car zacsCar = new Car(zac);

        //setting models
        myCar.setModel("Thrifty Mobile");
        lilysCar.setModel("Teal Jeep");
        bensCar.setModel("2013 Toyota Corrola");
        conradsCar.setModel("Ancient Ferrari");
        zacsCar.setModel("Brand New Tesla CyberTruck");


        myCar.accelerate(3.2);
        lilysCar.accelerate(5);

        //printing speed and models
        System.out.println("My car's speed: " + myCar.getSpeed() + "\nMy car's model: " + myCar.getModel());

        System.out.println("\nLily's car's speed: " + lilysCar.getSpeed() + "\nLily's car model: " + lilysCar.getModel());

        System.out.println("\nBen's car's speed: " + bensCar.getSpeed() + "\nBen's car model: " + bensCar.getModel());

        System.out.println("\nConrad's car's speed: "+ conradsCar.getSpeed() + "\nConrad's car model: " + conradsCar.getModel());

        System.out.println("\nZac's car's speed: " + zacsCar.getSpeed() + "\nZac's car model: " + zacsCar.getModel());


        //System.out.println("\n\n\nCurrent temperature is: " + zac.driverPreferedCarTemperature);
       
        //prefered temperatures
        macy.driverPreferedCarTemperature = 63;
        lily.driverPreferedCarTemperature = 69;
        ben.driverPreferedCarTemperature = 65;
        conrad.driverPreferedCarTemperature = 70;
        zac.driverPreferedCarTemperature = 68;

        //ac will go here


        //message about ac on
        myCar.acAndHeating();
       /*lilysCar.acAndHeating(); 
        bensCar.acAndHeating(); 
        conradsCar.acAndHeating();
        zacsCar.acAndHeating();*/
         
        //adjusting temperatures in cars
        //myCar.setTemperature(macy);
      /*  lilysCar.setTemperature(lily);
        bensCar.setTemperature(ben);
        conradsCar.setTemperature(conrad);
        zacsCar.setTemperature(zac);*/

        //System.out.println("Current temperature is: " + myCar.getTemperature() + "\nMy friend's car's temperature: " + lilysCar.getTemperature());
        
       /*  myCar.myRadio.getStation();
        System.out.println(myCar.myRadio.getStation());*/

       String currentGenre = GenreHandler.StationToGenre("100.3");
        System.out.println("\n" + currentGenre);


    }
}