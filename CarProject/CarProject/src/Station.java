public class Station {
    public double frequency;

    public Station(double frequency){

        //this.genre = genre;
        this.frequency = frequency;
    }

    @Override 
    public boolean equals(Object other){
        Station otherStation = (Station)other;
        return this.frequency == otherStation.frequency;
    }

    @Override
    public int hashCode(){
        return (int)Math.round(frequency * 10);
    } 
}