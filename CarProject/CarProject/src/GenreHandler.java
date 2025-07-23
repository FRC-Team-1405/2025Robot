import java.util.HashMap;

public class GenreHandler{
    public static String StationToGenre(String Station) {
        
        // a station object that we insert into a map
        Station myStation = new Station();
        myStation.frequency = 100.3;

        // create a new map to associate stations with genres
        HashMap<Station,Genre> StationGenre = new HashMap<Station,Genre>();

        // insert a station into the station/genre map
        StationGenre.put(myStation, Genre.COUNTRY);

        // take the input to this method, and create a new station to search through the map with
        Station frequencyToStation = new Station();
        frequencyToStation.frequency = Double.valueOf(Station);

        // search through the map and retrieve the associated genre
        Genre gottenGenre = StationGenre.get(frequencyToStation);

        // return the associated genre
        return gottenGenre.name();

    }
   
}