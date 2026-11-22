package xplane;

import javax.swing.*;
import java.awt.*;

// S.CHANGE - ZR
/*
    One always-on-top window holding the Flight Plan panel (left) and the Failure Injection
    panel (right), instead of two separate windows. The flight plan panel changes size when a
    long route wraps into more columns, so it asks this window to refit (refit()); the failure
    panel keeps its fixed width on the right.
*/
public class ControlWindow extends JFrame
{
    private static final int PREFERRED_Y = 40;   // where the failure window used to sit

    public static void open(FlightPlan plan, String xplaneRoot)
    {
        SwingUtilities.invokeLater(() -> new ControlWindow(plan, xplaneRoot).setVisible(true));
    }

    private ControlWindow(FlightPlan plan, String xplaneRoot)
    {
        super("Flight Plan & Failure Injection");

        JPanel right = new JPanel(new BorderLayout());   // keeps the failure panel at its own size
        right.add(new FailurePanel(), BorderLayout.NORTH);

        JPanel root = new JPanel(new BorderLayout(6, 0));
        root.add(new FlightPlanPanel(plan, xplaneRoot), BorderLayout.CENTER);
        root.add(right, BorderLayout.EAST);
        setContentPane(root);

        setDefaultCloseOperation(DISPOSE_ON_CLOSE);
        setAlwaysOnTop(true);
        setResizable(false);
        pack();
        setLocation(10, PREFERRED_Y);
        refit();
    }

    // Re-pack after the flight plan panel changed size, and keep the window on screen
    void refit()
    {
        pack();
        Rectangle screen = GraphicsEnvironment.getLocalGraphicsEnvironment().getMaximumWindowBounds();
        int x = getX() == 0 ? 10 : getX();
        x = Math.max(screen.x, Math.min(x, screen.x + screen.width - getWidth()));
        int y = Math.max(screen.y, Math.min(PREFERRED_Y, screen.y + screen.height - getHeight()));
        setLocation(x, y);
    }
}
// E.CHANGE - ZR
