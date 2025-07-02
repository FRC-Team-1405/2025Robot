public class App {
    public static void main(String[] args) throws Exception {
        System.out.println("1405");
        Car myCar = new Car();

        myCar.accelerate(3.2);
        System.out.println("My car's speed: " + myCar.getSpeed() + "\nMy car's model: " + myCar.getModel()); 

        Car myFriendsCar = new Car();

        myFriendsCar.setModel("Bronco");
        myFriendsCar.accelerate(5);

        System.out.println("My friend's car's speed: " + myFriendsCar.getSpeed() + "\nMy friend's car's model: " + myFriendsCar.getModel());

        System.out.println("My car's speed: " + myCar.getSpeed() + "\nMy car's model: " + myCar.getModel());
    
        Driver zac = new Driver();
        System.out.println("Current temperature is: " + zac.driverPreferedCarTemperature);

        Driver ben = new Driver();
        ben.driverPreferedCarTemperature = 65;
        zac.driverPreferedCarTemperature = 68;

        myCar.setTemperature(zac);
        myFriendsCar.setTemperature(ben);
        System.out.println("Current temperature is: " + myCar.getTemperature() + "\nMy friend's car's temperature: " + myFriendsCar.getTemperature());
        
        myCar.myRadio.getStation();
        System.out.println(myCar.myRadio.getStation());

    }
}