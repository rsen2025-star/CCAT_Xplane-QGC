package xplane;

import javax.swing.*;
import java.awt.*;

// S.CHANGE - ZR
public class FailurePanel extends JFrame
{
    // same order as FailureInjector.ALL_TARGETS
    private static final String[] LABELS = {"Lift 0", "Lift 1", "Lift 2", "Lift 3", "Pusher",
            "GPS", "IMU", "LIDAR", "Altimeter", "Radar alt"};
    private final JButton[] buttons = new JButton[FailureInjector.ALL_TARGETS.length];

    public static void open()
    {
        SwingUtilities.invokeLater(() -> new FailurePanel().setVisible(true));
    }

    public FailurePanel()
    {
        super("Failure Injection");
        JPanel grid = new JPanel(new GridLayout(5, 2, 6, 6));
        for (int i = 0; i < 5; i++)
        {
            grid.add(makeButton(i));       // engines on the left
            grid.add(makeButton(i + 5));   // sensors and altimeters on the right
        }
        JButton clear = new JButton("Clear all failures");
        clear.addActionListener(e -> new Thread(() -> FailureInjector.get().clearAll()).start());

        JPanel root = new JPanel(new BorderLayout(6, 6));
        root.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));
        root.add(grid, BorderLayout.CENTER);
        root.add(clear, BorderLayout.SOUTH);
        setContentPane(root);

        setDefaultCloseOperation(DISPOSE_ON_CLOSE);
        setAlwaysOnTop(true);
        setResizable(false);
        refresh();
        pack();
        setLocation(10, 40);
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
