import java.lang.reflect.Member;

public class App {
    public static void main(String[] args) throws Exception {
        int q = 5 ;
        // System.out.println ("hi") ;
        // System.out.println (2+1) ;
        // int x = 7-3 ;
        // int z = 60 ;
        // System.out.println (x);
        // if (x<100){
        //     System.out.println ("yay it worked!");
        // }
        // else{
        //     System.out.println (" it worked again! ") ;  
        // }
        // Car newCar = new Car () ;
        // System.out.println (newCar.gas) ;
        // newCar.fillTank(x) ;
        // System.out.println (newCar.gas) ;
        // //newCar.emptyTank(z) ;
        // System.out.println (newCar.gas); //TODO: use drive method 

        // newCar.drive(z) ;
        // System.out.println (newCar.gas);
        Car newCar1 = new car () ;
        newCar1.emptyTank(q) ; 
        System.out.println (newCar1.gas) ;


        Car newCar2 = new car () ;
        System.out.println (newCar2.gas) ;
     }
}