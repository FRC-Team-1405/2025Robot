import java.util.Random;

public class App {
    public static void main(String[] args) throws Exception {
        System.out.println("Hello, World!");
        Car myCar = new Car();
        Random random = new Random();
        int randomInt = random.ints(-20, 1).findFirst().getAsInt();
        myCar.exelorate (randomInt);
        System.out.println("Random number: " + randomInt);
        System.out.println( myCar. getspeed());
    }
}
