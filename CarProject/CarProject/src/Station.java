public class Station {
    private static Genre genre = Genre.COUNTRY;
    public double frequency = 92.5;

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