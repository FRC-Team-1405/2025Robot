public class Radio {
    
    public Radio(){
        GenreHandler.registerStation(new Station(76.5), Genre.REGGAE);
        GenreHandler.registerStation(new Station(83.6), Genre.SKA);
        GenreHandler.registerStation(new Station(96.5), Genre.ROCK);
        GenreHandler.registerStation(new Station(98.3), Genre.POP);
        GenreHandler.registerStation(new Station(100.3), Genre.COUNTRY);
        GenreHandler.registerStation(new Station(106.3), Genre.BROADWAY);
        GenreHandler.registerStation(new Station(127.0), Genre.CALYPSO);
        GenreHandler.registerStation(new Station(139.5), Genre.KLOVE);
        GenreHandler.registerStation(new Station(666.0), Genre.DEATH_METAL);

    }
    boolean on;
   // if(on = true){
   
        private Genre station = Genre.COUNTRY;
        public void setStation(Genre Station){
        station = Station;
        }
        
        public Genre getStation(){
            return station;
        }


   // }



}
