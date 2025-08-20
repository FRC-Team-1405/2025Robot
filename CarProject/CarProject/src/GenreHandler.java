import java.util.HashMap;
//import java.util.TreeMap;

public class GenreHandler{
    // create a new map to associate stations with genres
    static HashMap<Station,Genre> StationGenre = new HashMap<Station,Genre>();
    public static String StationToGenre(String Station) {
        
        // a station object that we insert into a map
         //Station myStation = new Station();

        //myStation.frequency = 100.3;

        

        // insert a station into the station/genre map
        //StationGenre.put(myStation, Genre.COUNTRY);
       
        Station myStationTwo = new Station(Double.valueOf(Station));

        //myStationTwo.frequency = Double.valueOf(Station);

    
        // search through the map and retrieve the associated genre
        Genre gottenGenre = StationGenre.get(myStationTwo);

        // return the associated genre
        return gottenGenre.name();

    }
    //to insert station into the map
    public static double registerStation(Station stationToRegister, Genre associatedGenre){
        StationGenre.put(stationToRegister, associatedGenre);
        return stationToRegister.frequency;
    }
}