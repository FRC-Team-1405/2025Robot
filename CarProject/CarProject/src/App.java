import java.lang.reflect.Member;

public class App {
    public static void main(String[] args) throws Exception {
        System.out.println ("hi") ;
        System.out.println (2+1) ;
        int x = 30-3 ;
        int z = 10 ;
        System.out.println (x);
        if (x<100){
            System.out.println ("yay it worked!");
        }
        else{
            System.out.println (" it worked again! ") ;  
        }
        Car newCar = new Car () ;
        System.out.println (newCar.gas) ;
        newCar.fillTank(x) ;
        System.out.println (newCar.gas) ;
        newCar.emptyTank(z) ;
        System.out.println (newCar.gas); //TODO: use drive method 
     }
}