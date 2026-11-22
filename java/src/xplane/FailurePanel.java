package xplane;

import javax.swing.*;
import java.awt.*;

// S.CHANGE - ZR
// Panel hosted by ControlWindow next to the Flight Plan panel (was its own always-on-top window)
public class FailurePanel extends JPanel
{
    // same order as FailureInjector.ALL_TARGETS
    private static final String[] LABELS = {"Lift 0", "Lift 1", "Lift 2", "Lift 3", "Pusher",
            "GPS", "IMU", "LIDAR", "Altimeter", "Radar alt"};
    private final JButton[] buttons = new JButton[FailureInjector.ALL_TARGETS.length];

    public FailurePanel()
    {
        super(new BorderLayout(6, 6));
        JPanel grid = new JPanel(new GridLayout(5, 2, 6, 6));
        for (int i = 0; i < 5; i++)
        {
            grid.add(makeButton(i));       // engines on the left
            grid.add(makeButton(i + 5));   // sensors and altimeters on the right
        }
        JButton clear = new JButton("Clear all failures");
        clear.addActionListener(e -> new Thread(() -> FailureInjector.get().clearAll()).start());

        setBorder(BorderFactory.createCompoundBorder(
                BorderFactory.createTitledBorder("Failure Injection"),
                BorderFactory.createEmptyBorder(8, 8, 8, 8)));
        add(grid, BorderLayout.CENTER);
        add(clear, BorderLayout.SOUTH);

        refresh();
        new Timer(250, e -> refresh()).start();
    }

    // each click cycles the target's mode; the injector may hit X-Plane, so keep it off the Swing thread
    private JButton makeButton(int i)
    {
        final String target = FailureInjector.ALL_TARGETS[i];
        JButton b = new JButton(LABELS[i]);
        b.setPreferredSize(new Dimension(190, 32));
        b.addActionListener(e -> new Thread(() -> FailureInjector.get().cycle(target)).start());
        buttons[i] = b;
        return b;
    }

    private void refresh()
    {
        for (int i = 0; i < buttons.length; i++)
        {
            FailureInjector.Mode mode = FailureInjector.get().mode(FailureInjector.ALL_TARGETS[i]);
            buttons[i].setText(LABELS[i] + ": " + mode);
            buttons[i].setForeground(mode == FailureInjector.Mode.OK ? Color.BLACK : Color.RED);
        }
    }
}
// E.CHANGE - ZR
