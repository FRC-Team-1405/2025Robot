import java.util.TreeMap;

public class GenreHandler{
    public static String StationToGenre(String Station) {
        
        // a station object that we insert into a map
         Station myStation = new Station();

        myStation.frequency = 100.3;

        // create a new map to associate stations with genres
        TreeMap<Station,Genre> StationGenre = new TreeMap<Station,Genre>();

        // insert a station into the station/genre map
        StationGenre.put(myStation, Genre.COUNTRY);
       
        Station myStationTwo = new Station();

        myStationTwo.frequency = Double.valueOf(Station);

    
        // search through the map and retrieve the associated genre
        Genre gottenGenre = StationGenre.get(myStationTwo);

        // return the associated genre
        return gottenGenre.name();

    }
   
}