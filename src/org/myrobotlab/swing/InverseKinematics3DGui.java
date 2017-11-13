package org.myrobotlab.swing;

import java.awt.Component;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.Box;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.JTextField;
import javax.swing.JTextPane;
import javax.swing.SwingUtilities;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.myrobotlab.kinematics.Point;
import org.myrobotlab.logging.LoggerFactory;
import org.myrobotlab.service.InverseKinematics3D;
import org.myrobotlab.service.SwingGui;
import org.slf4j.Logger;

/**
 * @author simlt
 */
public class InverseKinematics3DGui extends ServiceGui implements ActionListener {

  static final long serialVersionUID = 1L;
  public final static Logger log = LoggerFactory.getLogger(InverseKinematics3DGui.class);
  // private InverseKinematics3D ik;
  private Point tcpPos;
  private JTextField xPos;
  private JTextField yPos;
  private JTextField zPos;
  private FrameList selectedFrame;
  private double[][] tcpPosList;

  enum FrameList {
    WorldFrame,
    ArmFrame,
  }

  public InverseKinematics3DGui(final String boundServiceName, final SwingGui myService) {
    super(boundServiceName, myService);
    // ik = (InverseKinematics3D) Runtime.getService(boundServiceName);

    JPanel panel = new JPanel();
    display.add(panel);
    GridBagLayout gbl_panel = new GridBagLayout();
    gbl_panel.columnWidths = new int[] { 200, 16, 200, 0 };
    gbl_panel.rowHeights = new int[] { 64, 25, 157, 0 };
    gbl_panel.columnWeights = new double[] { 0.0, 1.0, 0.0, Double.MIN_VALUE };
    gbl_panel.rowWeights = new double[] { 0.0, 0.0, 0.0, Double.MIN_VALUE };
    panel.setLayout(gbl_panel);

    Insets insets = new Insets(5, 5, 5, 5);
    JButton btnZero = new JButton("Zero Joints");
    btnZero.setActionCommand("zeroAllJoints");
    btnZero.addActionListener(this);
    GridBagConstraints gbc_btnZero = new GridBagConstraints();
    gbc_btnZero.fill = GridBagConstraints.BOTH;
    gbc_btnZero.insets = insets;
    gbc_btnZero.gridx = 0;
    gbc_btnZero.gridy = 0;
    panel.add(btnZero, gbc_btnZero);
    JButton btnCenter = new JButton("Center Joints");
    btnCenter.setActionCommand("centerAllJoints");
    btnCenter.addActionListener(this);
    GridBagConstraints gbc_btnCenter = new GridBagConstraints();
    gbc_btnCenter.fill = GridBagConstraints.BOTH;
    gbc_btnCenter.insets = insets;
    gbc_btnCenter.gridx = 2;
    gbc_btnCenter.gridy = 0;
    panel.add(btnCenter, gbc_btnCenter);

    Box jointBox = Box.createVerticalBox();
    GridBagConstraints gbc_jointBox = new GridBagConstraints();
    gbc_jointBox.fill = GridBagConstraints.BOTH;
    gbc_jointBox.insets = insets;
    gbc_jointBox.gridx = 0;
    gbc_jointBox.gridy = 2;
    panel.add(jointBox, gbc_jointBox);

    /*JSlider slider = new JointSlider("omoplate");
    jointBox.add(slider);*/
    

    Box axisBox = Box.createVerticalBox();
    GridBagConstraints gbc_axisBox = new GridBagConstraints();
    gbc_axisBox.fill = GridBagConstraints.HORIZONTAL;
    gbc_axisBox.insets = insets;
    gbc_axisBox.gridx = 2;
    gbc_axisBox.gridy = 2;
    panel.add(axisBox, gbc_axisBox);

    JComboBox comboBox = new JComboBox(FrameList.values());
    comboBox.addActionListener(e -> {
      JComboBox cb = (JComboBox) e.getSource();
      selectedFrame = (FrameList) cb.getSelectedItem();
      updateTcpLabel();
    });
    comboBox.setSelectedIndex(0);
    axisBox.add(comboBox);

    Box posBox = Box.createHorizontalBox();
    axisBox.add(posBox);

    JLabel lblX = new JLabel("x");
    posBox.add(lblX);
    lblX.setLabelFor(xPos);
    xPos = new JTextField();
    posBox.add(xPos);

    Component horizontalStrut = Box.createHorizontalStrut(20);
    posBox.add(horizontalStrut);

    JLabel lblY = new JLabel("y");
    posBox.add(lblY);
    yPos = new JTextField();
    lblY.setLabelFor(yPos);
    posBox.add(yPos);

    Component horizontalStrut_1 = Box.createHorizontalStrut(20);
    posBox.add(horizontalStrut_1);

    JLabel lblZ = new JLabel("z");
    posBox.add(lblZ);
    zPos = new JTextField();
    lblZ.setLabelFor(zPos);
    posBox.add(zPos);

    Box horizontalBox = Box.createHorizontalBox();
    axisBox.add(horizontalBox);

    JButton btnx = new JButton("+X");
    horizontalBox.add(btnx);

    Component horizontalStrut_2 = Box.createHorizontalStrut(20);
    horizontalBox.add(horizontalStrut_2);

    JButton btny = new JButton("+Y");
    horizontalBox.add(btny);

    Component horizontalStrut_3 = Box.createHorizontalStrut(20);
    horizontalBox.add(horizontalStrut_3);

    JButton btnz = new JButton("+Z");
    horizontalBox.add(btnz);

    Box horizontalBoxN = Box.createHorizontalBox();
    axisBox.add(horizontalBoxN);

    JButton btnxN = new JButton("-X");
    horizontalBoxN.add(btnxN);

    Component horizontalStrut_4 = Box.createHorizontalStrut(20);
    horizontalBoxN.add(horizontalStrut_4);

    JButton btnyN = new JButton("-Y");
    horizontalBoxN.add(btnyN);

    Component horizontalStrut_5 = Box.createHorizontalStrut(20);
    horizontalBoxN.add(horizontalStrut_5);

    JButton btnzN = new JButton("-Z");
    horizontalBoxN.add(btnzN);

    btnx.setActionCommand(btnx.getText());
    btny.setActionCommand(btny.getText());
    btnz.setActionCommand(btnz.getText());
    btnxN.setActionCommand(btnxN.getText());
    btnyN.setActionCommand(btnyN.getText());
    btnzN.setActionCommand(btnzN.getText());
    btnx.addActionListener(this);
    btny.addActionListener(this);
    btnz.addActionListener(this);
    btnxN.addActionListener(this);
    btnyN.addActionListener(this);
    btnzN.addActionListener(this);
    
    myService.undockTab("ik3D");
  }

  @Override
  public void actionPerformed(ActionEvent e) {
    String command = e.getActionCommand();
    log.info("Parsing command string: " + command);
    int delta = 10;
    if (command.contains("-")) {
      delta = -delta;
    } else if (!command.contains("+")) {
      send(command);
      return;
    }
    if (tcpPos == null) {
      log.warn("TCP position is not set yet for moveTo");
      return;
    }
    double x = tcpPos.getX();
    double y = tcpPos.getY();
    double z = tcpPos.getZ();
    if (command.contains("X")) {
      x += delta;
    } else if (command.contains("Y")) {
      y += delta;
    } else if (command.contains("Z")) {
      z += delta;
    }

    String frame = "WORLD";
    if (selectedFrame.equals(FrameList.ArmFrame)) {
      frame = "BASE_ARM";
    } else if (selectedFrame.equals(FrameList.WorldFrame)) {
      frame = "WORLD";
    }

    log.info("Moving to " + x + " " + y + " " + z + " in frame: " + frame);
    myService.send(boundServiceName, "moveTo", x, y, z, frame);
  }

  @Override
  public void subscribeGui() {
    subscribe("publishJointPositions");
    subscribe("publishTcpPosition");
  }

  @Override
  public void unsubscribeGui() {
    unsubscribe("publishJointPositions");
    unsubscribe("publishTcpPosition");
  }

  public void onState(final InverseKinematics3D ik) {
    SwingUtilities.invokeLater(new Runnable() {
      @Override
      public void run() {

      }
    });
  }

  public void onJointPositions(double[][] jointMap) {
    /*
     * double[] p = jointMap[jointMap.length - 1]; tcpPos = new Point(p[0],
     * p[1], p[2], 0, 0, 0); xPos.setText(String.format("%.2f", tcpPos.getX()));
     * yPos.setText(String.format("%.2f", tcpPos.getY()));
     * zPos.setText(String.format("%.2f", tcpPos.getZ()));
     */
  }

  public void onTcpPosition(double[][] tcp) {
    tcpPosList = tcp;
    updateTcpLabel();
  }

  private void updateTcpLabel() {
    if (tcpPosList == null) {
      return;
    }
    int index;
    if (selectedFrame.equals(FrameList.ArmFrame)) {
      index = 0;
    } else {
      index = 1;
    }
    tcpPos = new Point(tcpPosList[index][0], tcpPosList[index][1], tcpPosList[index][2], 0, 0, 0);
    xPos.setText(String.format("%.2f", tcpPos.getX()));
    yPos.setText(String.format("%.2f", tcpPos.getY()));
    zPos.setText(String.format("%.2f", tcpPos.getZ()));
  }

  public class AxisJog extends JTextPane {
    private static final long serialVersionUID = 1L;

    int pos;
  }

  public class JointSlider extends JSlider implements ChangeListener {
    private static final long serialVersionUID = 1L;
    private String servoName;

    public JointSlider(/* InverseKinematics3D ik, */ String servoName) {
      this.servoName = servoName;
      // setMinimum(ik.);
      // setMaximum(maximum);
      addChangeListener(this);
    }

    @Override
    public void stateChanged(ChangeEvent e) {
      // TODO Auto-generated method stub
      JSlider slider = (JSlider) e.getSource();
      if (!slider.getValueIsAdjusting()) {
        slider.getValue();
      }
    }
  }

}
