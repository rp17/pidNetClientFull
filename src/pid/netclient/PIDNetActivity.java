package pid.netclient;

import gps.GPS;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.locks.Condition;

import java.net.UnknownHostException;
import java.io.IOException;

import pid.PIDControl;


import android.location.Criteria;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.provider.Settings;
import android.app.Activity;
import android.content.Intent;
import android.util.Log;
import android.view.Menu;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;
import android.os.PowerManager;
import android.view.Window;
import android.view.WindowManager;
import android.content.Context;
import android.content.res.Configuration;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

//import android.os.Looper;
//import android.os.Handler;
//import android.os.Message;

public class PIDNetActivity extends Activity implements LocationListener, SensorEventListener {
	// Here's the code that you'll use to initiate counters
	// tv_counter1.setText("counter 1:" + String.valueOf(INSERT_COUNTER1_VAR));
	// tv_counter2.setText("counter 2:" + String.valueOf(INSERT_COUNTER2_VAR));
	
	// Instantiate Textviews
	TextView tv_wp_start;
	TextView tv_wp_end;
	TextView tv_error;
	TextView tv_pidHeading;
	TextView tv_desired_course;
	TextView tv_current_course;
	TextView tv_current_comp_course;
	TextView tvLat;
	TextView tvLon;
	TextView tv_distance_from_end_wp;
	
	TextView tv_counter1;
	TextView tv_counter2;
	
	
	// Instantiate Buttons
	Button bt_wp_start;
	Button bt_wp_end;
	Button bt_myLocation;
	Button bt_manual_stop;
	
	// Instantiate GPS Data
	LocationManager locationManager;
	Criteria criteria;
	String bestProvider;
	Location location;
	LocationListener loc_listener;
	
	private static final int THREADNR = 2;
	private int SERVER_PORT = 5000;
	private String SERVER_IP = "192.168.1.4";
	
	// Instantiate Phone Latitude and Longitude
	// These will be the actual values from the GPS
	static volatile float lat;
	static volatile float lon;
	static volatile float azimut = 0.0f;
	static volatile float azimutPre = 0.0f;
	static final float azimutGain = 0.7f;
	static volatile int avgAzimut = 0;
	
	// Instantiate Start, End waypoints
	// also User current location
	static volatile float start_wp_lat;
	static volatile float start_wp_lon;
	static volatile float end_wp_lat;
	static volatile float end_wp_lon;
	static volatile float myCurrentLat;
	static volatile float myCurrentLon;
	
	// Instantiate desired course bearing,
	// current course bearing, crosstrack error,
	// and PID Heading
	static volatile float desireCourseBearing;
	static volatile float currentCourseBearing;
	static volatile int crossTrackError;
	static volatile int pidHeading;
	static volatile float distanceFromEndWayPoint = 0.0f;
	
	// Verify that both start and end 
	// waypoints have been recorded
	static volatile boolean AreAllTheWaypointsRecorded = false;
	
	// Confirm that 1 waypoint has been recorded
	static volatile boolean OneWaypointIsAlreadyRecorded = false;
	static volatile boolean EndWaypointIsAlreadyRecorded = false;
	// User will set to true if they press the manual
	// stop button
	//volatile boolean ManualStop = false;
	
	private SensorManager mSensorManager;
	Sensor accelerometer;
	Sensor magnetometer;
	  
	private volatile long lastLocListenerTime = System.nanoTime();
	private static PowerManager.WakeLock wl;
	private static final GPS gps = new GPS();
	private static final PIDControl pc = new PIDControl();
	
	//private static final ExecutorService pool = Executors.newCachedThreadPool();
	private static final ExecutorService pool = Executors.newFixedThreadPool(THREADNR);
	private static final ExecutorService singlePool = Executors.newSingleThreadExecutor();
	private static final ExecutorService singleCompPool = Executors.newSingleThreadExecutor();
	private static final ExecutorService singleClientPool = Executors.newSingleThreadExecutor();
	
	private final PIDLoop pidRunnable = new PIDNetActivity.PIDLoop();
	private IPIDClient clientLoop = new PID_UDP_Client(); 
	//private final CoordReader coordRunnable = new CoordReader();
	
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN, 
                                WindowManager.LayoutParams.FLAG_FULLSCREEN);
        setContentView(R.layout.activity_main);
    
        // Set up Text Views
        tv_wp_start = (TextView) findViewById(R.id.tv_startwp);
        tv_wp_end = (TextView) findViewById(R.id.tv_endwp);
        tv_error = (TextView) findViewById(R.id.tv_error);
        tv_pidHeading = (TextView) findViewById(R.id.tv_pidHeading);
        tv_desired_course = (TextView) findViewById(R.id.tv_desiredCourse);
        tv_current_course = (TextView) findViewById(R.id.tv_currentCourse);
        tv_current_comp_course = (TextView) findViewById(R.id.tv_currentCompCourse);
        tvLat = (TextView) findViewById(R.id.txtLat);
        tvLon = (TextView) findViewById(R.id.txtLon);
        tv_distance_from_end_wp = (TextView) findViewById(R.id.tv_distance_from_end_wp);
    	
        tv_counter1 = (TextView) findViewById(R.id.tv_counter1);
        tv_counter2 = (TextView) findViewById(R.id.tv_counter2);
        
        // Set up buttons and initiate their onclick listeners
        bt_wp_start = (Button) findViewById(R.id.bt_startwp);
        bt_wp_end = (Button) findViewById(R.id.bt_endwp);
        bt_myLocation = (Button) findViewById(R.id.bt_myLocation);
        bt_manual_stop = (Button) findViewById(R.id.bt_stop);
              
        bt_wp_start.setOnClickListener(wp_startHandler);
        bt_wp_end.setOnClickListener(wp_endHandler);
        bt_myLocation.setOnClickListener(wp_mylocationHandler);
        bt_manual_stop.setOnClickListener(wp_manualstopHandler);
        
        // Run Phone GPS to get latitude and longitude
        // GPS code done by: Jose Lopez
        // gpsSetup();
        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
    	accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    	magnetometer = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
    	mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME);
	    mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_GAME);
	    
         
        locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
      //check if the gps is enabled. if not ask the user to enable it.
  		boolean enabled = locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER);
  		if(!enabled){
  			Intent intent = new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS);
  			startActivity(intent);
  		}
  		
  		//setting the criteria to get the location
  		
  		criteria = new Criteria();
  		criteria.setAccuracy(Criteria.ACCURACY_FINE);
  		criteria.setAltitudeRequired(false);
  		criteria.setBearingRequired(false);
  		criteria.setCostAllowed(false);
  		criteria.setSpeedRequired(false);
  		//criteria.setPowerRequirement(Criteria.POWER_LOW);
  		
  		//adding criteria and best providers
  		bestProvider = locationManager.getBestProvider(criteria, false);
  		locationManager.requestLocationUpdates(bestProvider , 10, 0, this);
  		
        //locationManager.requestLocationUpdates( LocationManager.GPS_PROVIDER, 0, 0, this);
         
        // start the pid thread in the paused state
        //pidRunnable = new PIDLoop();
      
        pool.execute(pidRunnable);
        //pool.execute(coordRunnable);
        try {
        	boolean res = clientLoop.serverConnect(SERVER_IP, SERVER_PORT);
        	if(res) {
        		singleClientPool.execute(clientLoop);
        	}
        }
        catch(final UnknownHostException ex) {
        	runOnUiThread(new Runnable(){
   		 		@Override
   		 		public void run() {
   		 			// User is at the waypoint. Display an alert toast for a few seconds
   		 			Toast.makeText(getApplicationContext(), ex.getMessage(), Toast.LENGTH_SHORT).show();
   		 		}
   		 	});
        }
        catch(final IOException ex) {
        	runOnUiThread(new Runnable(){
   		 		@Override
   		 		public void run() {
   		 			// User is at the waypoint. Display an alert toast for a few seconds
   		 			Toast.makeText(getApplicationContext(), ex.getMessage(), Toast.LENGTH_SHORT).show();
   		 		}
   		 	});
        }
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        //PowerManager pm = (PowerManager) getSystemService(Context.POWER_SERVICE);
        //wl = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "PID control");
        //wl.acquire();
    }
    
    class CoordReader implements Runnable {
    	private int ct = 0;
    	private long lastTime = System.nanoTime();
    	public void run() {
    		while(true) {
    			long currTime = System.nanoTime();
    			try {
    				Thread.sleep(500);
    			}
    			catch(InterruptedException e) {}
    			if(currTime - lastTime > 100) {
    				lastTime = currTime;
    				runOnUiThread(new Runnable(){
    					@Override
    					public void run(){
    						tv_counter1.setText("counter 1: " + String.valueOf(ct));
    					}
    				});
    				ct++;
    			}
    			/*
    			if(locationManager != null) {
    				Location l = locationManager.getLastKnownLocation(bestProvider);
    				double latC = 0.0;
    				double lonC = 0.0;
      				try{
      					latC = l.getLatitude();
      					lonC = l.getLongitude();
      					//Log.e("GPS", "Location changed: lat=" + String.valueOf(lat) + " lon=" + String.valueOf(lon));
      					
      				} catch(NullPointerException e){
      					latC = -1.0;
      					lonC = -1.0;
      					Log.e("GPS Listener Exception", "" + e.getMessage());
      				}
      				final double latCf = latC;
      				final double lonCf = lonC;
      				runOnUiThread(new Runnable(){
      					@Override
      					public void run(){
      						tvLat.setText("Lat:" + String.valueOf(latCf) + " ");
      						tvLon.setText("Lon:" + String.valueOf(lonCf) + " ");
      					}
      				});
      				ct++;
    			}
    			*/
    		}
    	}
    }
    
    /** WE'LL NEED TO TAKE THAT WHILE LOOP THATS RUNNING IN THE 
     * wp_mylocationHandler AND INSERT IT HERE IN THIS THREAD
     */
    class PIDLoop implements Runnable {
    	private long lastTime = System.nanoTime();
    	private long lastPIDtime = lastTime;
    	private int ct = 0;
    	private ReentrantLock pauseLock = new ReentrantLock();
    	private Condition unpaused = pauseLock.newCondition();
    	// boolean "paused" is used to switch the thread between the wait and runnable states
    	// the thread starts in a paused state, waiting for the user input to start PID control
    	private volatile boolean paused = true;
    	// boolean "active" is used to terminate the thread, if "active" becomes false then the while loop inside run()
    	// finishes, so run() method finishes and the thread will enter the terminated state
    	public volatile boolean active = true; 
    	
    	public void run() {
    		Thread.currentThread().setPriority(Thread.MAX_PRIORITY);
    		while(active) {
    	  		
                // if manual stop has not been initiated by user then
                // the let the user know that PID algorithm is done processing
                // because user found their location
    			
    			//double latC = 0.0;
				//double lonC = 0.0;
    			long currTime = System.nanoTime();
    			
    			try {
    				Thread.sleep(10);
    			}
    			catch(InterruptedException e) {}
    			
    			/*
    			if(currTime - lastTime > 100) {
    				lastTime = currTime;
    				runOnUiThread(new Runnable(){
       		 			@Override
       		 			public void run() {
       		 				tv_counter2.setText("counter 2:" + String.valueOf(ct));
       		 			}
       		 		});
       		 		ct++;
    			}
    			*/
    			if(!AreAllTheWaypointsRecorded) {
    				// running on UI
           		 	runOnUiThread(new Runnable(){
           		 		@Override
           		 		public void run() {
           		 			// User is at the waypoint. Display an alert toast for a few seconds
           		 			Toast.makeText(getApplicationContext(), "Waypoints not set", Toast.LENGTH_SHORT).show();
           		 		}
           		 	});
    				onPause();
    			}
    			else {
    				/*
       				Location l = locationManager.getLastKnownLocation(bestProvider);
      				try{
      					latC = l.getLatitude();
      					lonC = l.getLongitude();
      					//Log.e("GPS", "Location changed: lat=" + String.valueOf(lat) + " lon=" + String.valueOf(lon));
      					
      				} catch(NullPointerException e){
      					latC = -1.0;
      					lonC = -1.0;
      					Log.e("GPS Listener Exception", "" + e.getMessage());
      				}
      				*/
      				//final double latCf = latC;
      				//final double lonCf = lonC;
    				//myCurrentLat = lat;
	                //myCurrentLon = lon;
    				distanceFromEndWayPoint = gps.get_gps_dist(myCurrentLat, myCurrentLon, end_wp_lat, end_wp_lon);
      				//distanceFromEndWayPoint = gps.get_gps_dist((float)latC, (float)lonC, (float)end_wp_lat, (float)end_wp_lon);
    				if(distanceFromEndWayPoint <= .5) {
    					// running on UI
    					runOnUiThread(new Runnable(){
    						@Override
    						public void run() {
    							// User is at the waypoint. Display an alert toast for a few seconds
    							Toast.makeText(getApplicationContext(), "You Found the Location", Toast.LENGTH_SHORT).show();
    						}
    					});
    						// the thread pauses itself if the destination is reached
    					pause();
    				}
    			}
    			pauseLock.lock();
    			try {
    				while (paused) unpaused.await();
    	        } 
    			catch (InterruptedException e) {
    	                    	runOnUiThread(new Runnable(){
    	        					@Override
    	        					public void run() {
    	        						Toast.makeText(getApplicationContext(), "Thread " + Thread.currentThread().getName() + " interrupted", Toast.LENGTH_SHORT).show();
    	        					}
    	        				});     
    	        } finally {
    	            pauseLock.unlock();
    	        }
    	            
                // The following while loop will need to be on a separate thread
                // possible use a ExecutorService thread
                // pool.execute(new PIDLoop())
                
                // Keep Running PID Algorithm until the user has issued a manual stop
                // or until user is near .1 distance of the end waivepoint
                //while(distanceFromEndWaivePoint > .1 && ManualStop == false) {
    			//myCurrentLat = lat;
                //myCurrentLon = lon;
                currentCourseBearing = gps.get_gps_course(myCurrentLat, myCurrentLon, end_wp_lat, end_wp_lon);
                //currentCourseBearing = gps.get_gps_course((float) latC, (float) lonC, (float)end_wp_lat, (float)end_wp_lon);
                crossTrackError = pc.compass_error((int)desireCourseBearing, (int) currentCourseBearing);
                pidHeading = pc.PID_heading(crossTrackError);
                	
              	// running on UI
                if(currTime - lastPIDtime > 10) {
    				lastPIDtime = currTime;
    				runOnUiThread(new Runnable(){
    					@Override
    					public void run() {
    						tv_current_course.setText(String.valueOf(currentCourseBearing));
    						tv_error.setText(String.valueOf(crossTrackError));
    						tv_pidHeading.setText(String.valueOf(pidHeading));
    					}
    				});       
                }
               // End of While Loop.
    		}
    	}
    	public void pause() {
    	     pauseLock.lock();
    	     try {
    	    	 runOnUiThread(new Runnable(){
 					@Override
 					public void run() {
 						// User is at the waypoint. Display an alert toast for a few seconds
 						Toast.makeText(getApplicationContext(), "Pausing PID", Toast.LENGTH_SHORT).show();
 					}
 				});
    	       paused = true;
    	     } finally {
    	       pauseLock.unlock();
    	     }
        }

        public void resume() {
        	pauseLock.lock();
            try {
            	runOnUiThread(new Runnable(){
					@Override
					public void run() {
						// User is at the waypoint. Display an alert toast for a few seconds
						Toast.makeText(getApplicationContext(), "Resuming PID", Toast.LENGTH_SHORT).show();
					}
				});
              paused = false;
              unpaused.signalAll();
            } finally {
              pauseLock.unlock();
            }
        }
    }
    
    // Record the Start Waypoint and get desired course only if
    // the end waypoint has already been determined
    View.OnClickListener wp_startHandler = new View.OnClickListener() {
        public void onClick(View v) {
           
        	// Take the Phone latitude and longitude values
        	// and store them in start waypoint
        	start_wp_lat = lat;
        	start_wp_lon = lon;
        	
        	// Display the latitude and longitude to the start waypoint textview
        	runOnUiThread(new Runnable(){
        	    @Override
        	    public void run() {
        	    	tv_wp_start.setText("lat: " + String.valueOf(start_wp_lat) +", " + "lon: " + String.valueOf(start_wp_lon));
        	    }
        	});
        	
           
        	// Set the desired course by passing in the start and end waypoints
        	// only if the end waypoint has already been recorded
        	if(OneWaypointIsAlreadyRecorded) {        		
        		// all the waypoints have been recorded now get desired course
        		AreAllTheWaypointsRecorded = true;
        		
        		desireCourseBearing = gps.get_gps_course((float)start_wp_lat, (float)start_wp_lon, (float)end_wp_lat, (float)end_wp_lon);
        		
        		runOnUiThread(new Runnable(){
        		    @Override
        		    public void run() {
        		    	tv_desired_course.setText(String.valueOf(desireCourseBearing));
        		    }
        		});
        	}
        	else
        		// else then record the start waypoint
        		OneWaypointIsAlreadyRecorded = true;
        }
      };
      
      // Record the End waypoint and get desired course only if
      // the start waypoint has already been determined
      View.OnClickListener wp_endHandler = new View.OnClickListener() {
          public void onClick(View v) {
        	
          	// Take the Phone latitude and longitude values
          	// and store them in end waypoint
        	end_wp_lat = lat;
        	end_wp_lon = lon;
        	
        	// Display the latitude and longitude to the end waypoint textview
        	runOnUiThread(new Runnable(){
        	    @Override
        	    public void run() {
        	    	tv_wp_end.setText("lat: " + String.valueOf(end_wp_lat) +", " + "lon: " + String.valueOf(end_wp_lon));
        	    }
        	});

        	EndWaypointIsAlreadyRecorded = true;
              
			// Set the desired course by passing in the start and end waypoints
			// only if the start waypoint has already been recorded
			if(OneWaypointIsAlreadyRecorded) {
				
        		// all the waypoints have been recorded now get desired course
        		AreAllTheWaypointsRecorded = true;
        		
        		desireCourseBearing = gps.get_gps_course((float)start_wp_lat, (float)start_wp_lon, (float)end_wp_lat, (float)end_wp_lon);
        		runOnUiThread(new Runnable(){
        		    @Override
        		    public void run() {
        		    	tv_desired_course.setText(String.valueOf(desireCourseBearing));
        		    }
        		});

				
			}
			else
				// else then record the end waypoint
				OneWaypointIsAlreadyRecorded = true;
              
          }
        };
        
        // Now at this point the desired course should already be determined
        // now begin to run algorithm and user should begin to navigate to 
        // end waypoint
        View.OnClickListener wp_mylocationHandler = new View.OnClickListener() {
            
        	public void onClick(View v) {
        		
        		if(AreAllTheWaypointsRecorded) {
        		
	                myCurrentLat = lat;
	                myCurrentLon = lon;
	         		 
	         		// Get the distance user is from waivepoint
	                //double LatdistanceFromEndwaypoint = Math.abs(myCurrentLat - end_wp_lat);
	                //double LondistanceFromEndwaypoint = Math.abs(myCurrentLon - end_wp_lon);
	                
	         		distanceFromEndWayPoint = gps.get_gps_dist((float)myCurrentLat, (float)myCurrentLon, (float)end_wp_lat, (float)end_wp_lon);
	         		
	          		// Running on UI
	         		 runOnUiThread(new Runnable(){
	         			@Override
	         			public void run() {
	                      	// Display the current latitude and longitude of the user
	         				tv_distance_from_end_wp.setText(String.valueOf(distanceFromEndWayPoint));
	         			}
	         		});
	         		pidRunnable.resume();
        		}
        		else {
        			// User needs to have a start and end waypoint setup in order to run PID algorithm
        			runOnUiThread(new Runnable(){
        			    @Override
        			    public void run() {
        			    	Toast.makeText(getApplicationContext(), "You need to have a start and end waypoint setup in order to run PID algorithm", Toast.LENGTH_SHORT).show();
        			    }
        			});
        		}
 	            	
            }
          };
          
          // Manually stop running PID algorithm and Stop GPS Listener
          View.OnClickListener wp_manualstopHandler = new View.OnClickListener() {
              public void onClick(View v) {
            	// User is at the waypoint. Display an alert toast for a few seconds
            	  runOnUiThread(new Runnable(){
            		    @Override
            		    public void run() {
            		    	Toast.makeText(getApplicationContext(), "Pausing PID thread", Toast.LENGTH_SHORT).show();  
            		    }
            		});
	            
            	  
            	// pause the PID thread. This will put it into wait state
            	pidRunnable.pause();
                
            	// Stop GPS 
            	//locationManager.removeUpdates(loc_listener);
              }
            };
 
  /*         
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.main, menu);
        return true;
    }
    */
            
    @Override
	public void onLocationChanged(final Location l){
    	singlePool.execute(new Runnable() {
            @Override
            public void run() {
            	try{
            		myCurrentLat = lat = (float)l.getLatitude();
            		myCurrentLon = lon = (float)l.getLongitude();
            		//Log.e("GPS", "Location changed: lat=" + String.valueOf(lat) + " lon=" + String.valueOf(lon));
            	} catch(NullPointerException e){
            		lat = -1.0f;
            		lon = -1.0f;
            		Log.e("GPS Listener Exception", "" + e.getMessage());
            	}
            	//long currentTime = System.nanoTime();
            	//if(currentTime - lastLocListenerTime > 100) {
            		//lastLocListenerTime = currentTime;
            	if(EndWaypointIsAlreadyRecorded) {
            		distanceFromEndWayPoint = gps.get_gps_dist(myCurrentLat, myCurrentLon, end_wp_lat, end_wp_lon);
            		runOnUiThread(new Runnable(){
            			@Override
            			public void run(){
            				tv_distance_from_end_wp.setText(String.valueOf(distanceFromEndWayPoint));
            			}
            		});
            	}
            		runOnUiThread(new Runnable(){
            			@Override
            			public void run(){
            				tvLat.setText("Lat:" + String.valueOf(lat) + " ");
            				tvLon.setText("Lon:" + String.valueOf(lon) + " ");
            			}
            		});
            	//}
            }
        });
	}
    
    @Override
	public void onProviderEnabled(String p){}
		
	@Override
	public void onProviderDisabled(String p){bestProvider = locationManager.getBestProvider(criteria, false);}

	@Override
	public void onStatusChanged(String provider, int status, Bundle extras){}
		
	@Override
	protected void onResume(){
		super.onResume();
		//locationManager.requestLocationUpdates(bestProvider, 0, 0, loc_listener);
		locationManager.requestLocationUpdates(bestProvider, 10, 0, this);
	    mSensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_GAME);
	    mSensorManager.registerListener(this, magnetometer, SensorManager.SENSOR_DELAY_GAME);
		//wl.acquire();
	}
    
	@Override
	protected void onPause(){
		super.onPause();
		//locationManager.removeUpdates(loc_listener);
		locationManager.removeUpdates(this);
		mSensorManager.unregisterListener(this);
		//wl.release();
	}
	
	@Override
	protected void onDestroy(){
		super.onDestroy();
		//locationManager.removeUpdates(loc_listener);
		//locationManager.removeUpdates(this);
		//shutdownAndAwaitTermination(pool);
		pool.shutdownNow();
		singlePool.shutdownNow();
		singleCompPool.shutdownNow();
	}
	/*
	@Override
	public void onConfigurationChanged(Configuration newConfig) {
		super.onConfigurationChanged(newConfig);
		if(newConfig.orientation == Configuration.ORIENTATION_LANDSCAPE){
		    Log.e("On Config Change","LANDSCAPE");
		}else {
		    Log.e("On Config Change","PORTRAIT");
		}
	}
	*/
	
	float[] mGravity;
	float[] mGeomagnetic;
	/*
	public void onSensorChanged(final SensorEvent event) {
	   	singleCompPool.execute(new Runnable() {
            @Override
            public void run() {
            	if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
            		mGravity = event.values;
            	if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
            		mGeomagnetic = event.values;
            	if (mGravity != null && mGeomagnetic != null) {
            		float R[] = new float[9];
            		float I[] = new float[9];
            		boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
            		if (success) {
            			float orientation[] = new float[3];
            			SensorManager.getOrientation(R, orientation);
            			azimut = orientation[0]; // orientation contains: azimut, pitch and roll
            			runOnUiThread(new Runnable(){
                			@Override
                			public void run(){
                				tv_current_comp_course.setText("Comp course:" + String.valueOf(azimut) + " ");
                			}
                		});
            		}
            	}
            }
	   	});
	}
	*/
	static boolean compFirstRun = true;
	static float[] azimBuf = new float[10];
	public void onSensorChanged(final SensorEvent event) {
		
           if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
            		mGravity = event.values;
            if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
            		mGeomagnetic = event.values;
            if (mGravity != null && mGeomagnetic != null) {
            		float R[] = new float[9];
            		float I[] = new float[9];
            boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
            		if (success) {
            			float orientation[] = new float[3];
            			SensorManager.getOrientation(R, orientation);
            			azimut = orientation[0]; // orientation contains: azimut, pitch and roll
            			if(compFirstRun) {
            				compFirstRun = false;
            				for(int i = 0; i<azimBuf.length; i++){
            					azimBuf[i] = azimut; 
            				}
            			}
            			else {
            				for(int i=0; i< (azimBuf.length - 1); i++) {
            					azimBuf[i] = azimBuf[i+1];
            				}
            				azimBuf[azimBuf.length - 1] = azimut;
            			}
            			float avg = 0.0f;
            			for(float x: azimBuf){
        					avg += x; 
        				}
            			avg *= 0.1f;
            			avgAzimut = (int)Math.toDegrees(avg);
            			//azimut = azimutPre*azimutGain + azimut*(1-azimutGain);
            			//azimutPre = azimut;
                		tv_current_comp_course.setText(String.valueOf(avgAzimut) + " ");
                		
            	}
            }
	   
	}
	public void onAccuracyChanged(Sensor sensor, int accuracy) {  }
	void shutdownAndAwaitTermination(ExecutorService pool) {
		   pool.shutdown(); // Disable new tasks from being submitted
		   try {
		     // Wait a while for existing tasks to terminate
		     if (!pool.awaitTermination(60, TimeUnit.SECONDS)) {
		       pool.shutdownNow(); // Cancel currently executing tasks
		       // Wait a while for tasks to respond to being cancelled
		       if (!pool.awaitTermination(60, TimeUnit.SECONDS)) System.err.println("Pool did not terminate");
		     }
		   }
		   catch (InterruptedException ie) {
		     // (Re-)Cancel if current thread also interrupted
		     pool.shutdownNow();
		     // Preserve interrupt status
		     Thread.currentThread().interrupt();
		   }
	 }
}
