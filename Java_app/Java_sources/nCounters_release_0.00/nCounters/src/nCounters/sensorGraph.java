package nCounters;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.Calendar;
import java.util.Scanner;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JTextField;

import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import com.fazecast.jSerialComm.SerialPort;  
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;


public class sensorGraph {
	
	static SerialPort selectedPort;
	static JProgressBar b;
	static int x = 0, timeWrite = 0;
	static String line, data;
	
	private static void customizeChart(JFreeChart chart) {
		XYPlot plot = chart.getXYPlot();
		plot.setOutlinePaint(Color.BLUE); 	    // sets paint color for plot outlines
	    plot.setBackgroundPaint(Color.BLACK);	// sets plot background		
	    plot.setRangeGridlinesVisible(true);	// sets paint color for the grid lines
	    plot.setRangeGridlinePaint(Color.WHITE);
	}
	
	public static void main(String[] args) {
	    DateFormat df1 = new SimpleDateFormat("dd/MM/yy HH:mm:ss");
	    Calendar calobj = Calendar.getInstance();
	    
		//create and configure the window
		JFrame window = new JFrame();
		window.setTitle("nCounter");
		window.setSize(1200,700);
		window.setLayout(new BorderLayout());
		window.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE); //This command tells if i close the program close my entire program
		
		b = new JProgressBar(0,100); //It is used to create a horizontal progress bar with the specified minimum and maximum value.
		b.setValue(0);
		b.setStringPainted(true);
		
		
		// create a drop down box and connect button, then place them at the top of the window
		JComboBox<String> portList = new JComboBox<String>();
		JButton connectButton = new JButton("Connect");
		JLabel label1 = new JLabel("Patient Name:");
		JTextField  textField1 = new JTextField(10);
		JLabel label2 = new JLabel("Target:");
		JTextField  textField2 = new JTextField(5);
		textField2.setText("0");
		JPanel topPanel = new JPanel();
		topPanel.add(label1);
		topPanel.add(textField1);
		topPanel.add(label2);
		topPanel.add(textField2);
		
		
		JLabel label3 = new JLabel("Battery level:");
		topPanel.add(label3);
		topPanel.add(b);
		topPanel.add(portList);
		topPanel.add(connectButton);
		window.add(topPanel, BorderLayout.NORTH);
		
		SerialPort[] portNames = SerialPort.getCommPorts();
		for (int i = 0; i < portNames.length; i++)
			portList.addItem(portNames[i].getSystemPortName());
		
		//create the line graph
		XYSeriesCollection dataset = new XYSeriesCollection();
		XYSeries series1 = new XYSeries("Weight value");
		XYSeries series2 = new XYSeries("Target line");
		dataset.addSeries(series1);
		dataset.addSeries(series2);
		
		//create a chart
		JFreeChart chart = ChartFactory.createXYLineChart("Total",
						"MS",
						"Weight",  
						dataset,  
						PlotOrientation.VERTICAL, false, false, false);		
		
		customizeChart(chart);
		window.add(new ChartPanel(chart), BorderLayout.CENTER);
		
		
		//configure the connect button and use another thread to listen for data
		connectButton.addActionListener(new ActionListener() {
			@Override public void actionPerformed (ActionEvent arg0) {
				if (connectButton.getText().equals("Connect")) {
					selectedPort = SerialPort.getCommPort(portList.getSelectedItem().toString());
					selectedPort.setComPortTimeouts(SerialPort.TIMEOUT_SCANNER,0,0);
					selectedPort.setBaudRate(9600);
					selectedPort.setNumDataBits(8);
					selectedPort.setNumStopBits(SerialPort.NO_PARITY);
					selectedPort.setParity(SerialPort.NO_PARITY);
					if(selectedPort.openPort()) {
						connectButton.setText("Disconnect");
						portList.setEnabled(false);
					}
					series1.clear();
					series2.clear();
					x = 0;
					
		            /******************
		             * 
		             * CREATE FOLDER
		             * 
		             ******************/
//					File f = new File(textField1.getText()); 
//					if (f.mkdirs() == true) 	{System.out.println("Directory Created");} 
//					else 						{System.out.println("Unsuccessful");}
					
					
					//create a new thread that listens for incoming text and populates the graph
					Thread thread = new Thread() {
						@Override public void run() {
							Scanner scanner = new Scanner(selectedPort.getInputStream());
							FileWriter fw = null;
				            BufferedWriter bw = null;
				            PrintWriter out = null;     
				            int battery=0;
				            int weight=0;
				            String[] arrSplit;
				            while(scanner.hasNextLine()) {
				            	try {
					            	line = scanner.nextLine();
					            	arrSplit = line.split(",");
					            	weight = Integer.parseInt(arrSplit[0]);
						            battery = Integer.parseInt(arrSplit[1]);
									series1.add(x, weight);
									if (battery <= 100)
										b.setValue(battery); //set bat volts

						            data = textField2.getText();
						            
						            //validation of target line
						            if (data.matches("[0-9]+")) {
							            series2.add(x, Integer.parseInt(data));		
						            } else {
							            series2.add(x, 0);
						            }
				            	} catch(Exception e) {
								      e.printStackTrace();
				            	}
				            	
					            
					            /******************
					             * 
					             * FILE HANDLING
					             * 
					             ******************/
					            try {
					            	String path = textField1.getText()+".txt";
//					            	String path = "//"+textField1.getText()+"//"+textField1.getText()+".txt";  //TODO: Moiz put the record the created directory, and do battery status work
//					            	System.out.println(path);
					                fw = new FileWriter(path, true);
					                bw = new BufferedWriter(fw);
					                out = new PrintWriter(bw);
					                if (timeWrite==0) {
							            out.println("TIMESTAMP (START) : " + df1.format(calobj.getTime()) + "\t\tTARGET LINE : " + data);
						            	System.out.println("data:"+ df1.format(calobj.getTime()));
					                	timeWrite = 1;
					                }
						            out.println(line + "\r\n");
					                out.close();
					            } catch (IOException e) {
								      e.printStackTrace();
					            }
				
					            /******************
					             * 
					             * DELAY HANDLING
					             * 
					             ******************/
								try {
									Thread.sleep(1);
								} catch(InterruptedException ex) {
								      ex.printStackTrace();
								}
								x++;
							}
							scanner.close();
						}
					};
					thread.start();
				} else {
					//disconnect from serial port
					selectedPort.closePort();
					portList.setEnabled(true);
					connectButton.setText("Connect");
			        try {
			        	ChartUtilities.saveChartAsPNG(new File(textField1.getText()+".png"), chart, 1200, 700);
					} catch (IOException e) {
						e.printStackTrace();
					}
			        //saveChartAsJPEG(textField1.getText()+"_Chart.png", chart, 1200, 700);
//					series1.clear();
//					series2.clear();
//					x = 0;
					timeWrite = 0;
				}
			}
		});
		
		//show the window
		window.setVisible(true);
	}
}