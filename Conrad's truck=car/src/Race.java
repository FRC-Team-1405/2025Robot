import java.util.Random;
import java.util.TreeMap;

public class Race {

    TreeMap<Car, Integer> carSpeedSorter = new TreeMap<>();

    public Race(int Cars){
        for(int i = 1; i <= Cars;){
            carSpeedSorter.put(new Car("Car "+ i++), 
            new Random().ints(0, 100).findFirst().getAsInt());
        }

    }

    public void run(){

        System.out.println("Race Results:\n" + carSpeedSorter);

    }

    
}