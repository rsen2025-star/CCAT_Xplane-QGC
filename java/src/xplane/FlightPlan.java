package xplane;

/*
    NEW FILE - BM
    A route of intermediate waypoints followed by the destination. The points are fed into
    WaypointController's destination slot one at a time, so the agent steers to the next
    point with the existing bearing-to-target. remainingKm() gives the distance along the
    rest of the route, which the agent uses as distance-to-target so it only starts its
    approach (land-at-target-km in flight-phases.soar) near the destination, not at each
    intermediate waypoint.
    The route comes from RoutePlanner (or the hard-coded fallback) and can be replaced from
    the Flight Plan window until takeoff is finished (replace()).
*/
public class FlightPlan {
    // A waypoint counts as reached within this distance, then the aircraft turns to the next.
    // Middle ground: closer than 1.5 km (which looked "hit" while still far away), but not so
    // close that sharp turns swing far past the corner (turn radius at cruise ~1 km).
    private static final double LEG_SWITCH_KM = 1.0;

    private double[][] points;         // intermediate waypoints, then the destination
    private int index = 0;             // point currently being flown to
    private WaypointController wc;     // set by start(), used by replace()

    private String departureName = "Departure", destinationName = "Destination";
    private String[] waypointNames = null;   // fix names for display (null = none)
    private double[] departurePos = null;    // where the route starts (aircraft start position)

    // False until a destination has been planned (the agent holds its takeoff until then)
    private volatile boolean routeSet = true;

    // No destination yet: only the departure (the aircraft's start position). The Flight Plan
    // window's Plan route fills in the route with replace().
    public FlightPlan(String departureName, double[] departurePos) {
        this(new double[0][], departurePos[0], departurePos[1]);
        this.departureName = departureName;
        this.destinationName = "";
        this.waypointNames = new String[0];
        this.departurePos = departurePos.clone();
        this.routeSet = false;
    }

    public boolean hasRoute() { return routeSet; }

    // From a generated route (RoutePlanner), with the fix name of each waypoint
    public FlightPlan(RoutePlanner.Route route) {
        this(route.departure, route.destination, route.waypoints,
                route.destinationPos[0], route.destinationPos[1]);
        this.waypointNames = route.names.clone();
        this.departurePos = route.departurePos.clone();
    }

    // Same as below, with airport names for display (e.g. "KMLB", "KMCO")
    public FlightPlan(String departureName, String destinationName,
                      double[][] intermediate, double destLat, double destLon) {
        this(intermediate, destLat, destLon);
        this.departureName = departureName;
        this.destinationName = destinationName;
    }

    public FlightPlan(double[][] intermediate, double destLat, double destLon) {
        points = new double[intermediate.length + 1][];
        for (int i = 0; i < intermediate.length; i++) {
            points[i] = new double[]{intermediate[i][0], intermediate[i][1]};
        }
        points[intermediate.length] = new double[]{destLat, destLon};
    }

    // Steers toward the current point (also used to resume the route after a Land-button abort)
    public synchronized void start(WaypointController wc) {
        this.wc = wc;
        if (!routeSet) {
            System.out.println("[FlightPlan] no destination yet - enter one in the Flight Plan window");
            return;
        }
        wc.setDestination(points[index][0], points[index][1]);
        log();
    }

    // New route from the Flight Plan window. Only before takeoff is finished: in cruise the
    // aircraft is already flying a leg of the old route. Returns false if it was too late.
    public synchronized boolean replace(RoutePlanner.Route route) {
        if (departed) return false;
        points = new double[route.waypoints.length + 1][];
        for (int i = 0; i < route.waypoints.length; i++) points[i] = route.waypoints[i].clone();
        points[route.waypoints.length] = route.destinationPos.clone();
        waypointNames = route.names.clone();
        departurePos = route.departurePos.clone();
        departureName = route.departure;
        destinationName = route.destination;
        index = 0;
        destinationReached = false;
        distanceToNextKm = Double.NaN;
        routeSet = true;
        System.out.println("[FlightPlan] new route " + departureName + " -> " + destinationName);
        if (wc != null) start(wc);
        return true;
    }

    // Distance (km) from the aircraft to the point being flown to, for the Flight Plan window
    private volatile double distanceToNextKm = Double.NaN;
    public double getDistanceToNextKm() { return distanceToNextKm; }

    // Moves on to the next point once close enough to the current one
    public synchronized void update(double lat, double lon, WaypointController wc) {
        double km = wc.getDistanceBetweenTwoPoints(lat, lon, points[index][0], points[index][1]);
        distanceToNextKm = km;
        if (index < points.length - 1 && km <= LEG_SWITCH_KM) {
            System.out.println(String.format("[FlightPlan] reached %s (%.5f, %.5f) at sensor position %.5f, %.5f - %.2f km away",
                    waypointNames != null ? waypointNames[index] : "WP" + (index + 1),
                    points[index][0], points[index][1], lat, lon, km));
            index++;
            distanceToNextKm = wc.getDistanceBetweenTwoPoints(lat, lon, points[index][0], points[index][1]);
            wc.setDestination(points[index][0], points[index][1]);
            log();
        }
    }

    // Distance (km) to the current point plus the remaining legs to the destination
    public synchronized double remainingKm(double lat, double lon, WaypointController wc) {
        double km = wc.getDistanceBetweenTwoPoints(lat, lon, points[index][0], points[index][1]);
        for (int i = index; i < points.length - 1; i++) {
            km += wc.getDistanceBetweenTwoPoints(points[i][0], points[i][1], points[i + 1][0], points[i + 1][1]);
        }
        return km;
    }

    // Landed on the destination (set by the agent when its flight phase becomes "landed")
    private volatile boolean destinationReached = false;

    public synchronized void markDestinationReached() {
        if (index == points.length - 1 && !destinationReached) {
            destinationReached = true;
            System.out.println("[FlightPlan] destination reached");
        }
    }

    public boolean isDestinationReached() { return destinationReached; }

    // Left the departure (set by the agent when it first switches to horizontal cruise)
    private volatile boolean departed = false;

    public synchronized void markDeparted() {
        if (!departed) {
            departed = true;
            System.out.println("[FlightPlan] departed");
        }
    }

    public boolean isDeparted() { return departed; }

    // Read-only views for FlightPlanPanel
    public synchronized int getIndex() { return index; }
    public synchronized String getDepartureName() { return departureName; }
    public synchronized double[] getDeparturePos() { return departurePos == null ? null : departurePos.clone(); }
    public synchronized String getDestinationName() { return destinationName; }
    public synchronized String[] getWaypointNames() { return waypointNames == null ? null : waypointNames.clone(); }

    public synchronized double[][] getPoints() {
        double[][] copy = new double[points.length][];
        for (int i = 0; i < points.length; i++) copy[i] = points[i].clone();
        return copy;
    }

    private void log() {
        System.out.println("[FlightPlan] leg " + (index + 1) + "/" + points.length
                + " -> " + points[index][0] + ", " + points[index][1]
                + (index == points.length - 1 ? " (destination)" : ""));
    }
}