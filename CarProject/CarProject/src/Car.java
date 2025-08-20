public class Car {
    double mpg = 30.5 ;
    double gas = 10 ;
    public double fillTank ( double gallonsToAdd ) {
        gas = gas + gallonsToAdd ; 
        return gas ;
    };
    public double emptyTank ( double gallonsToLoose) {
        gas = gas - gallonsToLoose ;
        return gas ;
    }
    public double drive ( double milesToDrive ) {
      double gallonsToLoose = milesToDrive / mpg ;
        emptyTank (gallonsToLoose); 
        return gas ;
    }
}