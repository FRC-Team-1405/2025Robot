import java.util.Random;

public class Car {

    private double speed = 0;
    public void exelorate (double pressure) {
        speed = pressure;
    }

    public double getspeed (){
        return speed;
    }
public boolean Crash() {
    Random randomcrash = new Random();
        int randomIntcrash = randomcrash.ints(0,101).findFirst().getAsInt();
        if (randomIntcrash < 13){
        return true;
        }
        return false;
}

}
