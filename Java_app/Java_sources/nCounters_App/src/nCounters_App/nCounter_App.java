package nCounters_App;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.Calendar;
import java.util.Scanner;

import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.SourceDataLine;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JProgressBar;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
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

public class nCounter_App 
{
	static SerialPort selectedPort;
	static JProgressBar b;
	static int x = 0, timeWrite = 0, cnt = 0;
	static String line, data;
	static int target_data = 0;
	static int beep = 0;
	public static float SAMPLE_RATE = 8000f;
    static JComboBox<String> feedbackComboBox;
	
	private static void customizeChart(JFreeChart chart) 
	{
		XYPlot plot = chart.getXYPlot();
		plot.setOutlinePaint(Color.BLUE); 	    // sets paint color for plot outlines
	    plot.setBackgroundPaint(Color.BLACK);	// sets plot background		
	    plot.setRangeGridlinesVisible(true);	// sets paint color for the grid lines
	    plot.setRangeGridlinePaint(Color.WHITE);
	}
	
	public static void tone(int hz, int msecs) 
	     throws LineUnavailableException 
	{
		tone(hz, msecs, 1.0);
	}

	public static void tone(int hz, int msecs, double vol)
	    throws LineUnavailableException 
	{
	  byte[] buf = new byte[1];
	  AudioFormat af = 
	      new AudioFormat(
	          SAMPLE_RATE, // sampleRate
	          8,           // sampleSizeInBits
	          1,           // channels
	          true,        // signed
	          false);      // bigEndian
	  SourceDataLine sdl = AudioSystem.getSourceDataLine(af);
	  sdl.open(af);
	  sdl.start();
	  for (int i=0; i < msecs*8; i++) 
	  {
	    double angle = i / (SAMPLE_RATE / hz) * 2.0 * Math.PI;
	    buf[0] = (byte)(Math.sin(angle) * 127.0 * vol);
	    sdl.write(buf,0,1);
	  }
	  sdl.drain();
	  sdl.stop();
	  sdl.close();
	}
	
	/* Main */
	public static void main(String[] args) 
	{	
		//create and configure the window
		JFrame window = new JFrame();
		window.setTitle("nCounters");
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
		
        // Create a JComboBox to select feedback option (Enable/Disable)
        feedbackComboBox = new JComboBox<>(new String[] {"Disable Feedback", "Enable Feedback"});
        feedbackComboBox.setSelectedIndex(0);  // Default is Disabled
		
		JPanel topPanel = new JPanel();
		topPanel.add(label1);
		topPanel.add(textField1);
		topPanel.add(label2);
		topPanel.add(textField2);
		
		
		JLabel label3 = new JLabel("Battery level:");
		topPanel.add(label3);
		topPanel.add(b);
		topPanel.add(feedbackComboBox);
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
						"Seconds",
						"Weight",  
						dataset,  
						PlotOrientation.VERTICAL, false, false, false);		
		
		customizeChart(chart);
		window.add(new ChartPanel(chart), BorderLayout.CENTER);
		
		//configure the connect button and use another thread to listen for data
		connectButton.addActionListener(new ActionListener() 
		{
			@Override public void actionPerformed (ActionEvent arg0) 
			{
		    	String Patient_name = textField1.getText().trim(); 
		    	int target_line = Integer.parseInt(textField2.getText()); 
		    	
				if (connectButton.getText().equals("Connect"))
				{
					if ((Patient_name.isEmpty()) ||
							(target_line == 0))
					{
						System.out.println ("Parameters are not filled by user\n");
						//TODO: Make text Field1 and text Field2 to Red as user haven't put a parameters
						
		                // Make Patient Name and Target textField red
		                textField1.setBackground(Color.RED);
		                textField2.setBackground(Color.RED);
					}
					else {
						//Make text field white back again.
		                textField1.setBackground(Color.WHITE);
		                textField2.setBackground(Color.WHITE);
		                
						selectedPort = SerialPort.getCommPort(portList.getSelectedItem().toString());
						selectedPort.setComPortTimeouts(SerialPort.TIMEOUT_SCANNER,0,0);
						selectedPort.setBaudRate(9600);
						selectedPort.setNumDataBits(8);
						selectedPort.setNumStopBits(SerialPort.NO_PARITY);
						selectedPort.setParity(SerialPort.NO_PARITY);
						if(selectedPort.openPort()) 
						{
							connectButton.setText("Disconnect");
							portList.setEnabled(false);
						}
						series1.clear();
						series2.clear();
						x = 0;
							
						//create a new thread that listens for incoming text and populates the graph
						Thread thread = new Thread() 
						{
							@Override public void run() 
							{
								Scanner scanner = new Scanner(selectedPort.getInputStream());
								FileWriter fw = null;
					            BufferedWriter bw = null;
					            PrintWriter out = null;     
					            int battery=0;
					            int weight=0;
					            String[] arrSplit;
					            while(scanner.hasNextLine()) 
					            {
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
							            if (data.matches("[0-9]+"))
							            {
							            	target_data = Integer.parseInt (data);						            							            	
							            	if ((weight > target_data) && 
							            			(target_data !=0) && 
							            			(beep == 0))
							            	{
		                                    	System.out.println("Beep triggered! Weight: " + weight + " exceeds target: " + target_data);
								                try {
								                	int freq = 2200;
								                	int millisec = 300;
								        			nCounter_App.tone(freq, millisec);
								        			System.out.println("beeped");
								        			try {
								        				Thread.sleep(10);
								        			} catch (InterruptedException e) {
								        				e.printStackTrace();
								        			}
								        		} catch (LineUnavailableException e) {
								        			e.printStackTrace();
								        		}
							            		
	                                            beep = 1;
							            	}
							            	else if (weight < target_data) 
							            	{
							            		beep = 0;
							            	}

							            	System.out.println (target_data);
								            series2.add(x, target_data);		
							            } 
							            else 
							            {
								            series2.add(x, 0);
							            }
							            
							            cnt++;
							            if (cnt > 10) 
							            {
							            	cnt = 0;
							            	x++;
							            }
							            
					            	} catch(Exception e) {
									      e.printStackTrace();
					            	}
					            	
						            /* File Handling */
						            try {
						        	    DateFormat df1 = new SimpleDateFormat("dd/MM/yy HH:mm:ss");
						        	    Calendar calobj = Calendar.getInstance();
						        	    String path = Patient_name +".txt";
						                fw = new FileWriter(Patient_name +".txt", true);
						                bw = new BufferedWriter(fw);
						                out = new PrintWriter(bw);
						                if (timeWrite==0) 
						                {
								            out.println("TIMESTAMP (START) : " + df1.format(calobj.getTime()) + "\t\tTARGET LINE : " + data);
							            	System.out.println("data:"+ df1.format(calobj.getTime()));
						                	timeWrite = 1;
						                }
							            out.println(line + "\r\n");
						                out.close();
						            } 
						            catch (IOException e) 
						            {
									      e.printStackTrace();
						            }
					
						            /* Delay Handling */
								}
								scanner.close();
							}
						};
						thread.start();
					}
				} 
				else 
				{
					/* Disconnect from serial port */
					selectedPort.closePort();
					portList.setEnabled(true);
					connectButton.setText("Connect");
					
                    // If feedback is enabled, show feedback dialog
                    if (feedbackComboBox.getSelectedIndex() == 1) {
                        // Create JTextArea for larger input
                        JTextArea textArea = new JTextArea(6, 40);  // 5 lines, 30 characters per line
                        JScrollPane scrollPane = new JScrollPane(textArea);  // Add JScrollPane for scrollable text area
                        int option = JOptionPane.showConfirmDialog(window, scrollPane, "Please provide feedback", JOptionPane.OK_CANCEL_OPTION, JOptionPane.PLAIN_MESSAGE);
                        
                        if (option == JOptionPane.OK_OPTION) {
                            String feedback = textArea.getText().trim();
                            if (!feedback.isEmpty()) {
                                // Store feedback in a text file
                                try (BufferedWriter writer = new BufferedWriter(new FileWriter(Patient_name +".txt", true))) {
                                    writer.write("Feedback: " + feedback + "\n");
                                } catch (IOException e) {
                                    e.printStackTrace();
                                }
                            }
                        }
                    }     
					
			        try {
			        	ChartUtilities.saveChartAsPNG(new File(textField1.getText()+".png"), chart, 1200, 700);
					} catch (IOException e) {
						e.printStackTrace();
					}
					timeWrite = 0;
				}
			}
		});
		
		//show the window
		window.setVisible(true);
	}
}
