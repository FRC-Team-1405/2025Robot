public class Radio {
    
    public Radio(){
        GenreHandler.registerStation();
        
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
