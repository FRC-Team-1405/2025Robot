public class App {
    public static void main(String[] args) {
        System.out.println("Hello, World!");

        Car myCar = new Car();
        myCar.accelerate(100);
        System.out.println(myCar.getSpeed() + "mph");

    }
}
