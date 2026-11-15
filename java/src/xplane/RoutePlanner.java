package xplane;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/*
    NEW FILE - BM
    Builds a flight plan from a departure and destination ICAO code using X-Plane's own
    navigation database (offline, same data the sim uses). The route is roughly direct:
    every SPACING_KM along the straight line it picks the closest real fix / VOR / NDB within
    CORRIDOR_KM, or the point on the line itself if there is none (a user waypoint, named WPTn
    after its position in the route).

    Files (under the X-Plane root; Custom Data/ is used instead of Resources/default data/
    when it has updated navdata):
      earth_fix.dat  named fixes        "lat lon IDENT ENRT|airport REGION ..."
      earth_nav.dat  navaids            "type lat lon elev freq range var IDENT ..." (2 NDB, 3 VOR)
      apt.dat        airports           "1|16|17 elev - - ICAO name", then "1302 datum_lat/lon"

    Check a route without X-Plane running:
      java -cp target/classes xplane.RoutePlanner KMLB KMCO
*/
public class RoutePlanner
{
    private static final double SPACING_KM = 15.0;       // about one waypoint per this distance
    private static final double CORRIDOR_KM = 5.0;       // a fix must be this close to the ideal point
    private static final double MIN_FROM_DEPARTURE_KM = 3.0;   // cruise starts after the vertical climb
    private static final double MIN_FROM_DESTINATION_KM = 4.0; // agent's approach starts at 3.5 km
    private static final double MIN_LEG_KM = 3.0;        // don't pick two points almost on top of each other
    private static final double VOR_BONUS_KM = 1.0;      // prefer a VOR over a fix that is slightly closer

    private final File root;
    private final WaypointController geo = new WaypointController();   // distance helper

    public static class Route
    {
        public final String departure, destination;
        public final double[] departurePos, destinationPos;
        public final double[][] waypoints;
        public final String[] names;
        public final double directKm;

        Route(String dep, String dest, double[] depPos, double[] destPos,
              double[][] waypoints, String[] names, double directKm)
        {
            this.departure = dep; this.destination = dest;
            this.departurePos = depPos; this.destinationPos = destPos;
            this.waypoints = waypoints; this.names = names; this.directKm = directKm;
        }
    }

    private static class Fix
    {
        final String name; final double lat, lon; final boolean vor;
        Fix(String name, double lat, double lon, boolean vor) { this.name = name; this.lat = lat; this.lon = lon; this.vor = vor; }
    }

    public RoutePlanner(String xplaneRoot)
    {
        this.root = new File(xplaneRoot);
    }

    public Route plan(String depIcao, String destIcao) throws IOException
    {
        double[][] airports = findAirports(depIcao, destIcao);
        if (airports[0] == null) throw new IOException("Airport " + depIcao + " not found in apt.dat");
        return planFrom(depIcao, airports[0], destIcao);
    }

    // Route from a fixed start position (where the aircraft is in X-Plane), labelled depLabel
    public Route planFrom(String depLabel, double[] dep, String destIcao) throws IOException
    {
        double[] dest = findAirports(destIcao, destIcao)[0];
        if (dest == null) throw new IOException("Airport " + destIcao + " not found in apt.dat");

        double directKm = geo.getDistanceBetweenTwoPoints(dep[0], dep[1], dest[0], dest[1]);

        // Flat local frame (km) around the departure: fine for routes up to a few hundred km
        double cosLat = Math.cos(Math.toRadians((dep[0] + dest[0]) / 2));
        double dx = (dest[1] - dep[1]) * 111.32 * cosLat, dy = (dest[0] - dep[0]) * 111.32;
        double len = Math.hypot(dx, dy);
        double ux = len > 0 ? dx / len : 0, uy = len > 0 ? dy / len : 0;

        List<Fix> fixes = loadFixes(dep, dest);

        List<double[]> points = new ArrayList<>();
        List<String> names = new ArrayList<>();
        int n = (int) Math.ceil(directKm / SPACING_KM);
        double prevAlong = 0;
        for (int k = 1; k < n; k++) {
            double f = (double) k / n;
            double idealLat = dep[0] + f * (dest[0] - dep[0]);
            double idealLon = dep[1] + f * (dest[1] - dep[1]);

            Fix best = null;
            double bestScore = Double.MAX_VALUE;
            for (Fix fx : fixes) {
                if (names.contains(fx.name)) continue;
                double d = geo.getDistanceBetweenTwoPoints(fx.lat, fx.lon, idealLat, idealLon);
                if (d > CORRIDOR_KM) continue;
                double along = ((fx.lon - dep[1]) * 111.32 * cosLat) * ux + ((fx.lat - dep[0]) * 111.32) * uy;
                if (along < prevAlong + MIN_LEG_KM || along < MIN_FROM_DEPARTURE_KM
                        || along > len - MIN_FROM_DESTINATION_KM) continue;   // no backtracking
                double score = d - (fx.vor ? VOR_BONUS_KM : 0);
                if (score < bestScore) { bestScore = score; best = fx; }
            }

            double lat, lon;
            String name;
            if (best != null) { lat = best.lat; lon = best.lon; name = best.name; }
            else { lat = idealLat; lon = idealLon; name = "WPT" + (points.size() + 1); }   // numbered by its position in the route

            double along = ((lon - dep[1]) * 111.32 * cosLat) * ux + ((lat - dep[0]) * 111.32) * uy;
            if (along < prevAlong + MIN_LEG_KM || along < MIN_FROM_DEPARTURE_KM
                    || along > len - MIN_FROM_DESTINATION_KM) continue;
            points.add(new double[]{lat, lon});
            names.add(name);
            prevAlong = along;
        }

        Route r = new Route(depLabel, destIcao, dep, dest,
                points.toArray(new double[0][]), names.toArray(new String[0]), directKm);
        print(r);
        return r;
    }

    // ICAO code of the airport / heliport closest to lat, lon, or null if none within maxKm.
    // Reads all of apt.dat (a few seconds).
    public String nearestAirport(double lat, double lon, double maxKm) throws IOException
    {
        String best = null, cur = null;
        double bestKm = maxKm, curLat = Double.NaN;
        try (BufferedReader r = new BufferedReader(new FileReader(aptFile()), 1 << 16)) {
            String line;
            while ((line = r.readLine()) != null) {
                if (line.startsWith("1 ") || line.startsWith("16 ") || line.startsWith("17 ")) {
                    String[] t = line.trim().split("\\s+");
                    cur = t.length > 4 ? t[4] : null;
                    curLat = Double.NaN;
                } else if (cur != null && line.startsWith("1302 datum_lat")) {
                    curLat = parseLast(line);
                } else if (cur != null && line.startsWith("1302 datum_lon") && !Double.isNaN(curLat)) {
                    double km = geo.getDistanceBetweenTwoPoints(lat, lon, curLat, parseLast(line));
                    if (km < bestKm) { bestKm = km; best = cur; }
                }
            }
        }
        return best;
    }

    private File aptFile()
    {
        File apt = new File(root, "Custom Scenery/Global Airports/Earth nav data/apt.dat");
        return apt.exists() ? apt : new File(root, "Global Scenery/Global Airports/Earth nav data/apt.dat");
    }

    // Datum (airport reference point) of both airports in one pass over apt.dat
    private double[][] findAirports(String a, String b) throws IOException
    {
        File apt = aptFile();
        double[][] found = new double[2][];
        double[][] runwaySum = new double[2][3];   // fallback: average of runway ends
        int cur = -1;
        try (BufferedReader r = new BufferedReader(new FileReader(apt), 1 << 16)) {
            String line;
            while ((line = r.readLine()) != null) {
                if (line.startsWith("1 ") || line.startsWith("16 ") || line.startsWith("17 ")) {
                    if (cur >= 0 && found[cur] == null && runwaySum[cur][2] > 0)
                        found[cur] = new double[]{runwaySum[cur][0] / runwaySum[cur][2], runwaySum[cur][1] / runwaySum[cur][2]};
                    if (found[0] != null && (found[1] != null || a.equalsIgnoreCase(b))) break;
                    String[] t = line.trim().split("\\s+");
                    String icao = t.length > 4 ? t[4] : "";
                    cur = icao.equalsIgnoreCase(a) ? 0 : icao.equalsIgnoreCase(b) ? 1 : -1;
                } else if (cur >= 0 && line.startsWith("1302 datum_lat")) {
                    double v = parseLast(line);
                    found[cur] = new double[]{v, found[cur] != null ? found[cur][1] : Double.NaN};
                } else if (cur >= 0 && line.startsWith("1302 datum_lon") && found[cur] != null) {
                    found[cur][1] = parseLast(line);
                } else if (cur >= 0 && line.startsWith("100 ")) {
                    // land runway: ... lat1 lon1 at columns 9,10 and lat2 lon2 at 18,19
                    String[] t = line.trim().split("\\s+");
                    if (t.length > 19) {
                        runwaySum[cur][0] += Double.parseDouble(t[9]) + Double.parseDouble(t[18]);
                        runwaySum[cur][1] += Double.parseDouble(t[10]) + Double.parseDouble(t[19]);
                        runwaySum[cur][2] += 2;
                    }
                }
            }
        }
        for (int i = 0; i < 2; i++)
            if (found[i] != null && Double.isNaN(found[i][1])) found[i] = null;
        return found;
    }

    // Enroute fixes, VORs and NDBs inside the route's bounding box (plus the corridor)
    private List<Fix> loadFixes(double[] dep, double[] dest) throws IOException
    {
        double pad = CORRIDOR_KM / 111.0 + 0.05;
        double minLat = Math.min(dep[0], dest[0]) - pad, maxLat = Math.max(dep[0], dest[0]) + pad;
        double padLon = pad / Math.max(0.2, Math.cos(Math.toRadians(dep[0])));
        double minLon = Math.min(dep[1], dest[1]) - padLon, maxLon = Math.max(dep[1], dest[1]) + padLon;

        List<Fix> fixes = new ArrayList<>();
        try (BufferedReader r = new BufferedReader(new FileReader(navFile("earth_fix.dat")), 1 << 16)) {
            String line;
            while ((line = r.readLine()) != null) {
                String[] t = line.trim().split("\\s+");
                if (t.length < 4 || !t[3].equals("ENRT")) continue;
                double lat, lon;
                try { lat = Double.parseDouble(t[0]); lon = Double.parseDouble(t[1]); } catch (NumberFormatException e) { continue; }
                if (lat >= minLat && lat <= maxLat && lon >= minLon && lon <= maxLon)
                    fixes.add(new Fix(t[2], lat, lon, false));
            }
        }
        try (BufferedReader r = new BufferedReader(new FileReader(navFile("earth_nav.dat")), 1 << 16)) {
            String line;
            while ((line = r.readLine()) != null) {
                String[] t = line.trim().split("\\s+");
                if (t.length < 8 || !(t[0].equals("2") || t[0].equals("3"))) continue;
                double lat, lon;
                try { lat = Double.parseDouble(t[1]); lon = Double.parseDouble(t[2]); } catch (NumberFormatException e) { continue; }
                if (lat >= minLat && lat <= maxLat && lon >= minLon && lon <= maxLon)
                    fixes.add(new Fix(t[7], lat, lon, t[0].equals("3")));
            }
        }
        return fixes;
    }

    private File navFile(String name)
    {
        File custom = new File(root, "Custom Data/" + name);
        return custom.exists() ? custom : new File(root, "Resources/default data/" + name);
    }

    private static double parseLast(String line)
    {
        String[] t = line.trim().split("\\s+");
        return Double.parseDouble(t[t.length - 1]);
    }

    private void print(Route r)
    {
        StringBuilder sb = new StringBuilder(String.format("[RoutePlanner] %s -> %s, %.1f km direct: %s",
                r.departure, r.destination, r.directKm, r.departure));
        for (int i = 0; i < r.names.length; i++)
            sb.append(String.format(" -> %s (%.5f, %.5f)", r.names[i], r.waypoints[i][0], r.waypoints[i][1]));
        sb.append(" -> ").append(r.destination);
        System.out.println(sb);
    }

    public static void main(String[] args) throws IOException
    {
        String root = args.length > 2 ? args[2] : "C:\\X-Plane 12";
        long t = System.currentTimeMillis();
        new RoutePlanner(root).plan(args[0], args[1]);
        System.out.println("(" + (System.currentTimeMillis() - t) + " ms)");
    }
}
