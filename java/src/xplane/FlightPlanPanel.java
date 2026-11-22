package xplane;

import javax.swing.*;
import javax.swing.text.AbstractDocument;
import javax.swing.text.AttributeSet;
import javax.swing.text.BadLocationException;
import javax.swing.text.DocumentFilter;
import java.awt.*;

/*
    NEW FILE - BM
    Flight plan window, similar to the proposed GUI in the presentation: departure box, the waypoints, destination box.
    The departure is locked to where the aircraft started in X-Plane (shown as the nearest
    airport, or coordinates). Type an ICAO code into the destination box and press Plan route
    (or Enter) to generate a new route from there with RoutePlanner. The route can only be
    changed until takeoff is finished; after that the destination is locked too.

    Black = where the aircraft is / is heading now, grey = still to come, green = done:
      departure    black outline until takeoff is finished, then green
      waypoints    the one being flown to is black, with the distance to it (in the
                   destination box on the final leg); filled + green once reached (within
                   FlightPlan's LEG_SWITCH_KM of it); later ones grey
      destination  grey outline, black on the final leg, green once landed there
*/
public class FlightPlanPanel extends JPanel   // S.E.CHANGE - ZR (was a JFrame; now hosted by ControlWindow)
{
    private static final Color BOX = new Color(22, 96, 128);
    private static final Color BOX_EDGE = new Color(14, 40, 56);
    private static final Color DONE = new Color(0, 168, 80);          // reached
    private static final Color NEXT = Color.BLACK;                    // heading to
    private static final Color REMAINING = new Color(170, 170, 170);  // still to come
    private static final Color ERROR = new Color(200, 40, 40);
    private static final int WIDTH = 428, PAD = 34, BOX_H = 56, ROW_H = 34, LIST_PAD = 22, MIN_LIST_H = 160;
    private static final int COL_W = 360;          // width of one waypoint column (fits the distance)
    private static final int OVERHEAD_H = 240;     // title bar, boxes, button, padding (everything but the list)
    // S.E.CHANGE - ZR (window placement moved to ControlWindow)
    private int rowsPerColumn = Integer.MAX_VALUE; // long routes wrap into more columns

    private enum State { REMAINING, NEXT, DONE }

    private final FlightPlan plan;
    private final String xplaneRoot;
    private final AirportBox departureBox, destinationBox;
    private final RouteListView list = new RouteListView();
    private final JButton planButton = new JButton("Plan route");
    private final JLabel status = new JLabel(" ");
    private boolean planning = false;

    // S.CHANGE - ZR
    // open() removed: ControlWindow creates this panel and hosts it
    public FlightPlanPanel(FlightPlan plan, String xplaneRoot)
    {
        super(new BorderLayout());
        // E.CHANGE - ZR
        this.plan = plan;
        this.xplaneRoot = xplaneRoot;

        departureBox = new AirportBox("Departure:", plan.getDepartureName(), false);
        destinationBox = new AirportBox("Destination:", plan.getDestinationName(), true);

        planButton.addActionListener(e -> planRoute());
        destinationBox.field.addActionListener(e -> planRoute());   // Enter
        status.setFont(new Font(Font.SANS_SERIF, Font.PLAIN, 13));

        JPanel controls = new JPanel(new BorderLayout(8, 0));
        controls.setOpaque(false);
        controls.add(planButton, BorderLayout.WEST);
        controls.add(status, BorderLayout.CENTER);

        JPanel bottom = new JPanel(new BorderLayout(0, 12));
        bottom.setOpaque(false);
        bottom.add(destinationBox, BorderLayout.NORTH);
        bottom.add(controls, BorderLayout.SOUTH);

        // S.CHANGE - ZR
        // This panel is the content now (it used to build a root panel for its own frame)
        setBackground(Color.WHITE);
        setBorder(BorderFactory.createEmptyBorder(PAD, PAD, 16, PAD));
        add(departureBox, BorderLayout.NORTH);
        add(list, BorderLayout.CENTER);
        add(bottom, BorderLayout.SOUTH);
        sizeList();
        if (!plan.hasRoute()) setStatus("No destination yet", false);

        new Timer(250, e -> refresh()).start();
        // E.CHANGE - ZR
    }

    // Generate a route from the typed codes, off the Swing thread (reading navdata takes ~2 s)
    private void planRoute()
    {
        if (planning) return;
        if (plan.isDeparted()) { setStatus("Route is locked after takeoff", true); return; }
        String dep = plan.getDepartureName(), dest = destinationBox.field.getText().trim();
        double[] depPos = plan.getDeparturePos();
        if (depPos == null) { setStatus("No start position (using the fallback route)", true); return; }
        if (!dest.matches("[A-Z0-9]{3,4}")) { setStatus("Enter a 3-4 letter airport code", true); return; }
        if (dep.equals(dest)) { setStatus("Destination is the departure", true); return; }

        planning = true;
        planButton.setEnabled(false);
        setStatus("Planning " + dep + " -> " + dest + "...", false);
        new Thread(() -> {
            String msg;
            boolean error;
            try {
                RoutePlanner.Route route = new RoutePlanner(xplaneRoot).planFrom(dep, depPos, dest);
                if (plan.replace(route)) {
                    msg = String.format("%d waypoints, %.0f km", route.waypoints.length, route.directKm);
                    error = false;
                } else {
                    msg = "Too late: takeoff already finished";
                    error = true;
                }
            } catch (Exception ex) {
                msg = ex.getMessage();
                error = true;
            }
            final String m = msg;
            final boolean err = error;
            SwingUtilities.invokeLater(() -> {
                planning = false;
                setStatus(m, err);
                destinationBox.field.setText(plan.getDestinationName());
                sizeList();
                // S.CHANGE - ZR
                Window host = SwingUtilities.getWindowAncestor(this);
                if (host instanceof ControlWindow) ((ControlWindow) host).refit();
                // E.CHANGE - ZR
                refresh();
            });
        }, "RoutePlanner").start();
    }

    private void setStatus(String text, boolean error)
    {
        status.setText(text);
        status.setForeground(error ? ERROR : Color.DARK_GRAY);
    }

    // Fit the list on screen: as many rows per column as the screen height allows, then more columns
    private void sizeList()
    {
        Rectangle screen = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
        int maxListH = screen.height - OVERHEAD_H;
        rowsPerColumn = Math.max(3, (maxListH - 2 * LIST_PAD) / ROW_H);

        int count = Math.max(plan.getPoints().length - 1, 1);
        int columns = (count + rowsPerColumn - 1) / rowsPerColumn;
        int rows = Math.min(count, rowsPerColumn);
        list.setPreferredSize(new Dimension(Math.max(WIDTH - 2 * PAD, columns * COL_W),
                Math.max(MIN_LIST_H, rows * ROW_H + 2 * LIST_PAD)));
    }

    // S.E.CHANGE - ZR (placeOnScreen removed: ControlWindow.refit() keeps the window on screen)

    private void refresh()
    {
        boolean departed = plan.isDeparted();
        int idx = plan.getIndex(), last = plan.getPoints().length - 1;
        departureBox.state = departed ? State.DONE : State.NEXT;
        destinationBox.state = plan.isDestinationReached() ? State.DONE
                : departed && idx == last ? State.NEXT : State.REMAINING;

        // On the final leg the destination is the next point: show the distance in its box
        double km = plan.getDistanceToNextKm();
        destinationBox.distance.setText(destinationBox.state == State.NEXT && !Double.isNaN(km)
                ? String.format("   %.1f km", km) : "");

        boolean editable = !departed && !planning;
        JTextField field = destinationBox.field;
        if (field.isEditable() != editable) {
            field.setEditable(editable);
            // Locked: no blinking caret, no focus, normal pointer instead of the text cursor
            field.setFocusable(editable);
            field.getCaret().setVisible(editable && field.isFocusOwner());
            field.setCursor(Cursor.getPredefinedCursor(editable ? Cursor.TEXT_CURSOR : Cursor.DEFAULT_CURSOR));
        }
        planButton.setEnabled(editable);
        if (departed && !status.getText().startsWith("Route is locked")) setStatus("Route is locked after takeoff", false);

        departureBox.repaint();
        destinationBox.repaint();
        list.repaint();
    }

    // Rounded box with a label and either a code field ("Destination: [KLAL]") or, when not
    // editable, fixed text ("Departure: KMCO")
    private static class AirportBox extends JPanel
    {
        final JTextField field = new JTextField(4);
        final JLabel distance = new JLabel("");   // distance when this is the point being flown to
        State state = State.REMAINING;

        AirportBox(String label, String code, boolean editable)
        {
            super(new GridBagLayout());
            setOpaque(false);
            setPreferredSize(new Dimension(WIDTH - 2 * PAD, BOX_H));
            Font font = new Font(Font.SANS_SERIF, Font.PLAIN, 18);

            JLabel l = new JLabel(label + " ");
            l.setFont(font);
            l.setForeground(Color.WHITE);

            field.setText(code);
            field.setFont(font.deriveFont(Font.BOLD));
            field.setForeground(Color.WHITE);
            field.setCaretColor(Color.WHITE);
            field.setOpaque(false);
            field.setBorder(BorderFactory.createMatteBorder(0, 0, 1, 0, new Color(255, 255, 255, 120)));
            field.setHorizontalAlignment(JTextField.CENTER);
            ((AbstractDocument) field.getDocument()).setDocumentFilter(new UpperCaseFilter(4));

            add(l);
            if (editable) {
                add(field);
            } else {
                JLabel value = new JLabel(code);
                value.setFont(font.deriveFont(Font.BOLD));
                value.setForeground(Color.WHITE);
                add(value);
            }
            distance.setFont(font.deriveFont(Font.BOLD, 15f));
            distance.setForeground(Color.WHITE);
            add(distance);
        }

        @Override
        protected void paintComponent(Graphics g0)
        {
            Graphics2D g = (Graphics2D) g0;
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            int w = getWidth(), h = getHeight();
            g.setColor(BOX);
            g.fillRoundRect(2, 2, w - 4, h - 4, 18, 18);
            g.setColor(state == State.DONE ? DONE : state == State.NEXT ? NEXT : REMAINING);
            g.setStroke(new BasicStroke(4));
            g.drawRoundRect(2, 2, w - 4, h - 4, 18, 18);
        }
    }

    // Upper-case letters/digits only, at most max characters
    private static class UpperCaseFilter extends DocumentFilter
    {
        private final int max;
        UpperCaseFilter(int max) { this.max = max; }

        @Override
        public void replace(FilterBypass fb, int offset, int length, String text, AttributeSet attrs) throws BadLocationException
        {
            String clean = text == null ? "" : text.toUpperCase().replaceAll("[^A-Z0-9]", "");
            int room = max - (fb.getDocument().getLength() - length);
            if (clean.length() > room) clean = clean.substring(0, Math.max(room, 0));
            super.replace(fb, offset, length, clean, attrs);
        }

        @Override
        public void insertString(FilterBypass fb, int offset, String text, AttributeSet attr) throws BadLocationException
        {
            replace(fb, offset, 0, text, attr);
        }
    }

    // The waypoint rows between the two boxes
    private class RouteListView extends JPanel
    {
        RouteListView() { setOpaque(false); }

        @Override
        protected void paintComponent(Graphics g0)
        {
            Graphics2D g = (Graphics2D) g0;
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);

            double[][] points = plan.getPoints();
            String[] names = plan.getWaypointNames();
            int idx = plan.getIndex();
            int waypointCount = points.length - 1;
            boolean departed = plan.isDeparted();

            Font rowFont = new Font(Font.SANS_SERIF, Font.PLAIN, 16);
            Font rowBold = rowFont.deriveFont(Font.BOLD);
            g.setFont(rowFont);
            int y = LIST_PAD;
            if (!plan.hasRoute()) {
                g.setColor(Color.GRAY);
                g.drawString("Takeoff Halted", 12, y + 22);
                g.drawString("Enter a destination and press Plan route to start", 12, y + 44);
            } else if (waypointCount == 0) {
                g.setColor(Color.GRAY);
                g.drawString("Direct to destination", 12, y + 22);
            }
            // Waypoint rows: reached = filled + green, heading to = black, later = grey.
            // Top to bottom, then on into the next column.
            for (int i = 0; i < waypointCount; i++) {
                int col = i / rowsPerColumn, row = i % rowsPerColumn;
                int cx = 12 + col * COL_W, cy = y + row * ROW_H + 8, d = 18;
                boolean reached = i < idx;
                boolean next = departed && i == idx;
                if (reached) {
                    g.setColor(BOX);
                    g.fillOval(cx, cy, d, d);
                }
                g.setColor(reached ? BOX_EDGE : next ? NEXT : REMAINING);
                g.setStroke(new BasicStroke(2));
                g.drawOval(cx, cy, d, d);
                g.setColor(reached ? DONE : next ? NEXT : REMAINING);
                int tx = cx + d + 14;
                if (names != null && i < names.length) {
                    g.setFont(rowBold);
                    g.drawString(names[i], tx, cy + 15);
                    tx += 70;
                    g.setFont(rowFont);
                }
                String coords = String.format("%.5f, %.5f", points[i][0], points[i][1]);
                g.drawString(coords, tx, cy + 15);
                // Distance to the waypoint being flown to (only that one, to keep the list clean)
                double km = plan.getDistanceToNextKm();
                if (next && !Double.isNaN(km)) {
                    g.setFont(rowBold);
                    g.drawString(String.format("%.1f km", km), tx + g.getFontMetrics(rowFont).stringWidth(coords) + 12, cy + 15);
                    g.setFont(rowFont);
                }
            }
        }
    }
}
