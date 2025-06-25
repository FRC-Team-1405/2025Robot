import java.util.Scanner;


public class App {
    public static void main(String[] args) throws InterruptedException {

        Car myCar = new Car();
        myCar.accelerate(100);
        System.out.println(myCar.getSpeed() + "mph");


        Stopwatch time = new Stopwatch();
        Stopwatch elapse = new Stopwatch();
        time.start();

        Scanner scanner = new Scanner(System.in);
        System.out.println("Enter 'Stop' to stop");


        while (time.startTime != 0) {

            elapse.start();

            String stop = "Stop";
            Thread.sleep(2000);
            System.out.println(elapse.getElapsedTime());
            String input = scanner.nextLine();
            System.out.println(myCar.getSpeed() + "mph");
            if (input.equals(stop)) {
                time.stop();
            
            }

            elapse.stop();
            System.out.println();

            if (time.running == false) {
                break;
            }
        }

        System.out.println(time.getElapsedTime());

    }
}
