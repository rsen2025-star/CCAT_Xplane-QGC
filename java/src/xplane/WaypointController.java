package xplane;

public class WaypointController {
    public Waypoint waypoints[];
    public int currentWaypoint = 3;

    private double EARTH_RADIUS = 6371.0;
    private double DISTANCE_TO_WAYPOINT_THRESHOLD = 0.5; // When the plane is within this distance of the target waypoint, it will switch to the next waypoint

    public WaypointController(){
        waypoints = new Waypoint[7];
        waypoints[0] = new Waypoint(36.30,-115.17);          // route
        waypoints[1] = new Waypoint(36.40,-115.17);          // route
        waypoints[2] = new Waypoint(37.2,-115.17);           // route
        waypoints[3] = new Waypoint(38.1,-115.17,true);      // default landing zone (no auto-advance)

        waypoints[4] = new Waypoint(36.22688293457031,-115.04660034179688, true); // option 1 Landing
        waypoints[5] = new Waypoint(36.21802520751953,-115.13361358642578, true); // option 2 Landing
        waypoints[6] = new Waypoint(36.117820739746094,-115.17686462402344, true); // option 3 Landing
    }

    // S.CHANGE - BM
    // Replaces the default landing zone with the given coordinates and steers toward it
    public synchronized void setDestination(double lat, double lon){
        waypoints[3] = new Waypoint(lat, lon, true);
        currentWaypoint = 3;
        System.out.println("[WaypointController] Destination set to " + lat + ", " + lon);
    }

    // Sets the destination km ahead of the given position along headingDeg (true), so the
    // agent's normal approach can land there (used by the pilot's Land button)
    public synchronized void setDestinationAhead(double lat, double lon, double headingDeg, double km){
        double lat1 = Math.toRadians(lat), lon1 = Math.toRadians(lon);
        double brng = Math.toRadians(headingDeg), d = km / EARTH_RADIUS;
        double lat2 = Math.asin(Math.sin(lat1) * Math.cos(d) + Math.cos(lat1) * Math.sin(d) * Math.cos(brng));
        double lon2 = lon1 + Math.atan2(Math.sin(brng) * Math.sin(d) * Math.cos(lat1),
                Math.cos(d) - Math.sin(lat1) * Math.sin(lat2));
        setDestination(Math.toDegrees(lat2), Math.toDegrees(lon2));
    }
    // E.CHANGE - BM
    public void updateWaypoints(double lat, double lon){
        // If plane is within threshold of waypoint, then update waypoint to next point
        if (getDistanceToWaypoint(lat,lon) <= waypoints[currentWaypoint].threshold) {
            currentWaypoint = (currentWaypoint + 1) % waypoints.length;
        }
    }
    public double calculateHeading(double lat, double lon){
        double lat2 = waypoints[currentWaypoint].getLat(),lon2 = waypoints[currentWaypoint].getLon();
        lat = Math.toRadians(lat);
        lat2 = Math.toRadians(lat2);
        lon = Math.toRadians(lon);
        lon2 = Math.toRadians(lon2);

        double Y = Math.cos(lat2) * Math.sin(lon2-lon);
        double X = (Math.cos(lat) * Math.sin(lat2)) - (Math.sin(lat) * Math.cos(lat2) *Math.cos(lon2-lon));
        double heading_angle = Math.atan2(Y,X);
        heading_angle = Math.toDegrees(heading_angle);

        // Autopilot expects a compass heading of 0-360
        return (heading_angle + 360.0) % 360.0; // S.E.CHANGE - BM
    }

    public double getDistanceToWaypoint(double lat, double lon){

        double lat2 = waypoints[currentWaypoint].getLat(),lon2 = waypoints[currentWaypoint].getLon();
        lat = Math.toRadians(lat);
        lat2 = Math.toRadians(lat2);
        lon = Math.toRadians(lon);
        lon2 = Math.toRadians(lon2);

        /* Haversine formula to calculate great-circle distance*/
        double distance = 0;
        distance = Math.pow(Math.sin((lon-lon2)/2),2);
        distance *= Math.cos(lat)*Math.cos(lat2);
        distance += Math.pow(Math.sin((lat-lat2)/2),2);
        distance = Math.sqrt(distance);
        distance = Math.asin(distance);
        distance *= 2 * EARTH_RADIUS;

        return distance;
    }
    // Calculates distance between two GPS coordinates using the Haversine formula
    public double getDistanceBetweenTwoPoints (double lat, double lon, double lat2, double lon2){

        lat = Math.toRadians(lat);
        lat2 = Math.toRadians(lat2);
        lon = Math.toRadians(lon);
        lon2 = Math.toRadians(lon2);

        /* Haversine formula to calculate great-circle distance*/
        double distance = 0;
        distance = Math.pow(Math.sin((lon-lon2)/2),2);
        distance *= Math.cos(lat)*Math.cos(lat2);
        distance += Math.pow(Math.sin((lat-lat2)/2),2);
        distance = Math.sqrt(distance);
        distance = Math.asin(distance);
        distance *= 2 * EARTH_RADIUS;

        return distance;
    }

    public class Waypoint{
        private double lat, lon;
        public boolean isLandingPoint = false;
        public double threshold = DISTANCE_TO_WAYPOINT_THRESHOLD;
        public Waypoint(double lat, double lon){
            this.lat = lat;
            this.lon = lon;
        }
        public Waypoint(double lat, double lon, boolean isLandingPoint){
            this.lat = lat;
            this.lon = lon;
            this.isLandingPoint = isLandingPoint;
            this.threshold = -1;
        }
        public double getLat(){
            return lat;
        }
        public double getLon(){
            return lon;
        }



    }

}
