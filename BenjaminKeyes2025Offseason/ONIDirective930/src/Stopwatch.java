public class Stopwatch {
    public long startTime;
    private long endTime;
    public boolean running;

    // Start the stopwatch
    public void start() {
        if (!running) {
            startTime = System.nanoTime();
            running = true;
            System.out.println("Stopwatch started.");
        } else {
            System.out.println("Stopwatch is already running.");
        }
    }

    // Stop the stopwatch
    public void stop() {
        if (running) {
            endTime = System.nanoTime();
            running = false;
            System.out.println("Stopwatch stopped.");
        } else {
            System.out.println("Stopwatch is not running.");
        }
    }

    // Get elapsed time in seconds
    public double getElapsedTime() {
        if (running) {
            return (System.nanoTime() - startTime) / 1_000_000_000.0;
        } else {
            return (endTime - startTime) / 1_000_000_000.0;
        }
    }

    public static void main(String[] args) throws InterruptedException {
        Stopwatch stopwatch = new Stopwatch();

        stopwatch.start();
        // Simulate some task
        Thread.sleep(2000); // 2 seconds
        stopwatch.stop();

        System.out.printf("Elapsed time: %.2f seconds%n", stopwatch.getElapsedTime());
    }
}
