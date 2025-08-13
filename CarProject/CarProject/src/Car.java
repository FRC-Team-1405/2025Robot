public class Car {

    private double speed = 0;

    private String model = "Ford";

    private int carTemperature = 0;


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

    public Radio myRadio = new Radio();
}