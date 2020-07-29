/*
 *  Author: Amanjot Gulshi
 *  Email: amanjot@korechi.com
 *  Description: Android Studio code written for Korechi Android App to interact with and
 *  control RoamIO mobile robotic platform
 */

/*
 *  Author: Amanjot Gulshi
 *  Email: amanjot@korechi.com
 *  Description: Android Studio code written for Korechi Android App to interact with and
 *  control RoamIO mobile robotic platform
 */

package com.korechi.roamiocloud.views;

import android.content.Context;
import android.graphics.Point;
import android.util.AttributeSet;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.animation.Animation;
import android.view.animation.AnimationSet;
import android.view.animation.LinearInterpolator;
import android.view.animation.RotateAnimation;
import android.view.animation.ScaleAnimation;
import android.widget.ImageView;
import android.widget.RelativeLayout;
import android.widget.TextView;

import com.korechi.roamiocloud.R;

import org.ros.android.android_core_components.R.id;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.util.Timer;
import java.util.TimerTask;

import geometry_msgs.Twist;
import nav_msgs.Odometry;

public class VirtualJoystickView extends RelativeLayout implements Animation.AnimationListener, MessageListener<Odometry>, NodeMain {
    private float magnetTheta = 10.0F;
    private Publisher<Twist> publisher;
    private RelativeLayout mainLayout;
    private ImageView intensity;
    private ImageView thumbDivet;
    private ImageView lastVelocityDivet;
    private ImageView[] orientationWidget;
    private TextView magnitudeText;
    private float contactTheta;
    private float normalizedMagnitude;
    private float contactRadius;
    private float deadZoneRatio = 0.0F / 0.0F;
    private float joystickRadius = 0.0F / 0.0F;
    private float parentSize = 0.0F / 0.0F;
    private float normalizingMultiplier;
    private ImageView currentRotationRange;
    private ImageView previousRotationRange;
    private volatile boolean turnInPlaceMode;
    private float turnInPlaceStartTheta = 0.0F / 0.0F;
    private float rightTurnOffset;
    private volatile float currentOrientation;
    private int pointerId = -1;
    private Point contactUpLocation;
    private boolean previousVelocityMode;
    private boolean magnetizedYAxis;
    private boolean holonomic;
    private volatile boolean publishVelocity;
    private Timer publisherTimer;
    public Twist currentVelocityCommand;
    private String topicName;

    public VirtualJoystickView(Context context) {
        super(context);
        this.initVirtualJoystick(context);
        this.topicName = "~cmd_vel";
    }

    public VirtualJoystickView(Context context, AttributeSet attrs) {
        super(context, attrs);
        this.initVirtualJoystick(context);
        this.topicName = "~cmd_vel";
    }

    public VirtualJoystickView(Context context, AttributeSet attrs, int defStyle) {
        super(context, attrs, defStyle);
        this.topicName = "~cmd_vel";
    }

    public void setHolonomic(boolean enabled) {
        this.holonomic = enabled;
    }

    public void onAnimationEnd(Animation animation) {
        this.contactRadius = 0.0F;
        this.normalizedMagnitude = 0.0F;
        this.updateMagnitudeText();
    }

    public void onAnimationRepeat(Animation animation) {
    }

    public void onAnimationStart(Animation animation) {
    }

    public void onNewMessage(Odometry message) {
        double w = message.getPose().getPose().getOrientation().getW();
        double x = message.getPose().getPose().getOrientation().getX();
        double y = message.getPose().getPose().getOrientation().getZ();
        double z = message.getPose().getPose().getOrientation().getY();
        double heading = Math.atan2(2.0D * y * w - 2.0D * x * z, x * x - y * y - z * z + w * w) * 180.0D / 3.141592653589793D;
        this.currentOrientation = (float) (-heading);
        if (this.turnInPlaceMode) {
            this.post(new Runnable() {
                public void run() {
                    VirtualJoystickView.this.updateTurnInPlaceRotation();
                }
            });
            this.postInvalidate();
        }

    }

    public boolean onTouchEvent(MotionEvent event) {
        int action = event.getAction();
        switch (action & 255) {
            case 0:
                this.pointerId = event.getPointerId(event.getActionIndex());
                this.onContactDown();
                if (this.inLastContactRange(event.getX(event.getActionIndex()), event.getY(event.getActionIndex()))) {
                    this.previousVelocityMode = true;
                    this.onContactMove((float) this.contactUpLocation.x + this.joystickRadius, (float) this.contactUpLocation.y + this.joystickRadius);
                } else {
                    this.onContactMove(event.getX(event.getActionIndex()), event.getY(event.getActionIndex()));
                }
                break;
            case 1:
            case 6:
                if ((action & '\uff00') >> 8 == this.pointerId) {
                    this.onContactUp();
                }
                break;
            case 2:
                if (this.pointerId != -1) {
                    if (this.previousVelocityMode) {
                        if (this.inLastContactRange(event.getX(event.getActionIndex()), event.getY(event.getActionIndex()))) {
                            this.onContactMove((float) this.contactUpLocation.x + this.joystickRadius, (float) this.contactUpLocation.y + this.joystickRadius);
                        } else {
                            this.previousVelocityMode = false;
                        }
                    } else {
                        this.onContactMove(event.getX(event.findPointerIndex(this.pointerId)), event.getY(event.findPointerIndex(this.pointerId)));
                    }
                }
            case 3:
            case 4:
            case 5:
        }

        return true;
    }

    public void EnableSnapping() {
        this.magnetTheta = 20.0F;
    }

    public void DisableSnapping() {
        this.magnetTheta = 1.0F;
    }

    protected void onLayout(boolean changed, int l, int t, int r, int b) {
        super.onLayout(changed, l, t, r, b);
        if (this.mainLayout.getWidth() != this.mainLayout.getHeight()) {
            this.setOnTouchListener(null);
        }

        this.parentSize = (float) this.mainLayout.getWidth();
        if (this.parentSize < 200.0F || this.parentSize > 400.0F) {
            this.setOnTouchListener(null);
        }

        this.joystickRadius = (float) (this.mainLayout.getWidth() / 2);
        this.normalizingMultiplier = 1.363636F / (this.parentSize / 2.0F);
        this.deadZoneRatio = 16.5F * this.normalizingMultiplier;
        this.magnitudeText.setTextSize(this.parentSize / 12.0F);
    }

    private void animateIntensityCircle(float endScale) {
        AnimationSet intensityCircleAnimation = new AnimationSet(true);
        intensityCircleAnimation.setInterpolator(new LinearInterpolator());
        intensityCircleAnimation.setFillAfter(true);
        RotateAnimation rotateAnim = new RotateAnimation(this.contactTheta, this.contactTheta, this.joystickRadius, this.joystickRadius);
        rotateAnim.setInterpolator(new LinearInterpolator());
        rotateAnim.setDuration(0L);
        rotateAnim.setFillAfter(true);
        intensityCircleAnimation.addAnimation(rotateAnim);
        ScaleAnimation scaleAnim = new ScaleAnimation(this.contactRadius, endScale, this.contactRadius, endScale, this.joystickRadius, this.joystickRadius);
        scaleAnim.setDuration(0L);
        scaleAnim.setFillAfter(true);
        intensityCircleAnimation.addAnimation(scaleAnim);
        this.intensity.startAnimation(intensityCircleAnimation);
    }

    private void animateIntensityCircle(float endScale, long duration) {
        AnimationSet intensityCircleAnimation = new AnimationSet(true);
        intensityCircleAnimation.setInterpolator(new LinearInterpolator());
        intensityCircleAnimation.setFillAfter(true);
        intensityCircleAnimation.setAnimationListener(this);
        RotateAnimation rotateAnim = new RotateAnimation(this.contactTheta, this.contactTheta, this.joystickRadius, this.joystickRadius);
        rotateAnim.setInterpolator(new LinearInterpolator());
        rotateAnim.setDuration(duration);
        rotateAnim.setFillAfter(true);
        intensityCircleAnimation.addAnimation(rotateAnim);
        ScaleAnimation scaleAnim = new ScaleAnimation(this.contactRadius, endScale, this.contactRadius, endScale, this.joystickRadius, this.joystickRadius);
        scaleAnim.setDuration(duration);
        scaleAnim.setFillAfter(true);
        intensityCircleAnimation.addAnimation(scaleAnim);
        this.intensity.startAnimation(intensityCircleAnimation);
    }

    private void animateOrientationWidgets() {
        for (int i = 0; i < this.orientationWidget.length; ++i) {
            float deltaTheta = this.differenceBetweenAngles((float) (i * 15), this.contactTheta);
            if (deltaTheta < 40.0F) {
                this.orientationWidget[i].setAlpha(1.0F - deltaTheta / 40.0F);
            } else {
                this.orientationWidget[i].setAlpha(0.0F);
            }
        }

    }

    private float differenceBetweenAngles(float angle0, float angle1) {
        return Math.abs((angle0 + 180.0F - angle1) % 360.0F - 180.0F);
    }

    private void endTurnInPlaceRotation() {
        this.turnInPlaceMode = false;
        this.currentRotationRange.setAlpha(0.0F);
        this.previousRotationRange.setAlpha(0.0F);
        this.intensity.setAlpha(1.0F);
    }

    private void initVirtualJoystick(Context context) {
        this.setGravity(17);
        LayoutInflater.from(context).inflate(R.layout.virtual_joystick, this, true);
        this.mainLayout = this.findViewById(id.virtual_joystick_layout);
        this.magnitudeText = this.findViewById(id.magnitude);
        this.intensity = this.findViewById(id.intensity);
        this.thumbDivet = this.findViewById(id.thumb_divet);
        this.orientationWidget = new ImageView[24];
        this.orientationWidget[0] = this.findViewById(id.widget_0_degrees);
        this.orientationWidget[1] = this.findViewById(id.widget_15_degrees);
        this.orientationWidget[2] = this.findViewById(id.widget_30_degrees);
        this.orientationWidget[3] = this.findViewById(id.widget_45_degrees);
        this.orientationWidget[4] = this.findViewById(id.widget_60_degrees);
        this.orientationWidget[5] = this.findViewById(id.widget_75_degrees);
        this.orientationWidget[6] = this.findViewById(id.widget_90_degrees);
        this.orientationWidget[7] = this.findViewById(id.widget_105_degrees);
        this.orientationWidget[8] = this.findViewById(id.widget_120_degrees);
        this.orientationWidget[9] = this.findViewById(id.widget_135_degrees);
        this.orientationWidget[10] = this.findViewById(id.widget_150_degrees);
        this.orientationWidget[11] = this.findViewById(id.widget_165_degrees);
        this.orientationWidget[12] = this.findViewById(id.widget_180_degrees);
        this.orientationWidget[13] = this.findViewById(id.widget_195_degrees);
        this.orientationWidget[14] = this.findViewById(id.widget_210_degrees);
        this.orientationWidget[15] = this.findViewById(id.widget_225_degrees);
        this.orientationWidget[16] = this.findViewById(id.widget_240_degrees);
        this.orientationWidget[17] = this.findViewById(id.widget_255_degrees);
        this.orientationWidget[18] = this.findViewById(id.widget_270_degrees);
        this.orientationWidget[19] = this.findViewById(id.widget_285_degrees);
        this.orientationWidget[20] = this.findViewById(id.widget_300_degrees);
        this.orientationWidget[21] = this.findViewById(id.widget_315_degrees);
        this.orientationWidget[22] = this.findViewById(id.widget_330_degrees);
        this.orientationWidget[23] = this.findViewById(id.widget_345_degrees);
        ImageView[] var2 = this.orientationWidget;
        int var3 = var2.length;

        int var4;
        ImageView tack;
        for (var4 = 0; var4 < var3; ++var4) {
            tack = var2[var4];
            tack.setAlpha(0.0F);
            tack.setVisibility(View.INVISIBLE);
        }

        this.magnitudeText.setTranslationX((float) (40.0D * Math.cos((double) (90.0F + this.contactTheta) * 3.141592653589793D / 180.0D)));
        this.magnitudeText.setTranslationY((float) (40.0D * Math.sin((double) (90.0F + this.contactTheta) * 3.141592653589793D / 180.0D)));
        this.animateIntensityCircle(0.0F);
        this.contactTheta = 0.0F;
        this.animateOrientationWidgets();
        this.currentRotationRange = this.findViewById(id.top_angle_slice);
        this.previousRotationRange = this.findViewById(id.mid_angle_slice);
        this.currentRotationRange.setAlpha(0.0F);
        this.previousRotationRange.setAlpha(0.0F);
        this.lastVelocityDivet = this.findViewById(id.previous_velocity_divet);
        this.contactUpLocation = new Point(0, 0);
        this.holonomic = false;
        var2 = this.orientationWidget;
        var3 = var2.length;

        for (var4 = 0; var4 < var3; ++var4) {
            tack = var2[var4];
            tack.setVisibility(View.INVISIBLE);
        }

    }

    private void onContactDown() {
        this.thumbDivet.setAlpha(1.0F);
        this.magnitudeText.setAlpha(1.0F);
        this.lastVelocityDivet.setAlpha(0.0F);
        ImageView[] var1 = this.orientationWidget;
        int var2 = var1.length;

        for (int var3 = 0; var3 < var2; ++var3) {
            ImageView tack = var1[var3];
            tack.setVisibility(View.VISIBLE);
        }

        this.publishVelocity = true;
    }

    private void onContactMove(float x, float y) {
        float thumbDivetX = x - this.joystickRadius;
        float thumbDivetY = y - this.joystickRadius;
        this.contactTheta = (float) (Math.atan2(thumbDivetY, thumbDivetX) * 180.0D / 3.141592653589793D + 90.0D);
        this.contactRadius = (float) Math.sqrt(thumbDivetX * thumbDivetX + thumbDivetY * thumbDivetY) * this.normalizingMultiplier;
        this.normalizedMagnitude = (this.contactRadius - this.deadZoneRatio) / (1.0F - this.deadZoneRatio);
        if (this.contactRadius >= 1.0F) {
            thumbDivetX /= this.contactRadius;
            thumbDivetY /= this.contactRadius;
            this.normalizedMagnitude = 1.0F;
            this.contactRadius = 1.0F;
        } else if (this.contactRadius < this.deadZoneRatio) {
            thumbDivetX = 0.0F;
            thumbDivetY = 0.0F;
            this.normalizedMagnitude = 0.0F;
        }

        if (!this.magnetizedYAxis) {
            if ((this.contactTheta + 360.0F) % 180.0F < this.magnetTheta) {
                this.contactTheta -= (this.contactTheta + 360.0F) % 180.0F;
            } else if ((this.contactTheta + 360.0F) % 180.0F > 180.0F - this.magnetTheta) {
                this.contactTheta += 180.0F - (this.contactTheta + 360.0F) % 180.0F;
            }

            if (this.floatCompare(this.contactTheta, 180.0F) || this.floatCompare(this.contactTheta, 360.0F)) {
                this.magnetizedYAxis = true;
            }
        } else if (this.differenceBetweenAngles((this.contactTheta + 360.0F) % 360.0F, 180.0F) < 20.0F) {
            this.contactTheta = 180.0F;
        } else if (this.differenceBetweenAngles((this.contactTheta + 360.0F) % 360.0F, 360.0F) < 20.0F) {
            this.contactTheta = 360.0F;
        } else {
            this.magnetizedYAxis = false;
        }

        this.animateIntensityCircle(this.contactRadius);
        this.animateOrientationWidgets();
        this.updateThumbDivet(thumbDivetX, thumbDivetY);
        this.updateMagnitudeText();
        /*if (this.holonomic) {
            this.publishVelocity((double) this.normalizedMagnitude * Math.cos((double) this.contactTheta * 3.141592653589793D / 180.0D), (double) this.normalizedMagnitude * Math.sin((double) this.contactTheta * 3.141592653589793D / 180.0D), 0.0D);
        } else {
            this.publishVelocity((double) this.normalizedMagnitude * Math.cos((double) this.contactTheta * 3.141592653589793D / 180.0D), 0.0D, (double) this.normalizedMagnitude * Math.sin((double) this.contactTheta * 3.141592653589793D / 180.0D));
        }*/

        this.updateTurnInPlaceMode();
    }

    private void updateTurnInPlaceMode() {
        if (!this.turnInPlaceMode) {
            if (this.floatCompare(this.contactTheta, 270.0F)) {
                this.turnInPlaceMode = true;
                this.rightTurnOffset = 0.0F;
            } else {
                if (!this.floatCompare(this.contactTheta, 90.0F)) {
                    return;
                }

                this.turnInPlaceMode = true;
                this.rightTurnOffset = 15.0F;
            }

            this.initiateTurnInPlace();
            (new Timer()).schedule(new TimerTask() {
                public void run() {
                    VirtualJoystickView.this.post(new Runnable() {
                        public void run() {
                            if (VirtualJoystickView.this.turnInPlaceMode) {
                                VirtualJoystickView.this.currentRotationRange.setAlpha(1.0F);
                                VirtualJoystickView.this.previousRotationRange.setAlpha(1.0F);
                                VirtualJoystickView.this.intensity.setAlpha(0.2F);
                            }

                        }
                    });
                    VirtualJoystickView.this.postInvalidate();
                }
            }, 200L);
        } else if (!this.floatCompare(this.contactTheta, 270.0F) && !this.floatCompare(this.contactTheta, 90.0F)) {
            this.endTurnInPlaceRotation();
        }

    }

    private void onContactUp() {
        this.animateIntensityCircle(0.0F, (long) (this.normalizedMagnitude * 1000.0F));
        this.magnitudeText.setAlpha(0.4F);
        this.lastVelocityDivet.setTranslationX(this.thumbDivet.getTranslationX());
        this.lastVelocityDivet.setTranslationY(this.thumbDivet.getTranslationY());
        this.lastVelocityDivet.setAlpha(0.4F);
        this.contactUpLocation.x = (int) this.thumbDivet.getTranslationX();
        this.contactUpLocation.y = (int) this.thumbDivet.getTranslationY();
        this.updateThumbDivet(0.0F, 0.0F);
        this.pointerId = -1;
//        this.publishVelocity(0.0D, 0.0D, 0.0D);
        this.publishVelocity = false;
//        this.publisher.publish(this.currentVelocityCommand);
        this.endTurnInPlaceRotation();
        ImageView[] var1 = this.orientationWidget;
        int var2 = var1.length;

        for (int var3 = 0; var3 < var2; ++var3) {
            ImageView tack = var1[var3];
            tack.setVisibility(View.INVISIBLE);
        }

    }

    /*private void publishVelocity(double linearVelocityX, double linearVelocityY, double angularVelocityZ) {
        this.currentVelocityCommand.getLinear().setX(linearVelocityX);
        this.currentVelocityCommand.getLinear().setY(-linearVelocityY);
        this.currentVelocityCommand.getLinear().setZ(0.0D);
        this.currentVelocityCommand.getAngular().setX(0.0D);
        this.currentVelocityCommand.getAngular().setY(0.0D);
        this.currentVelocityCommand.getAngular().setZ(-angularVelocityZ);
    }*/

    private void initiateTurnInPlace() {
        this.turnInPlaceStartTheta = (this.currentOrientation + 360.0F) % 360.0F;
        RotateAnimation rotateAnim = new RotateAnimation(this.rightTurnOffset, this.rightTurnOffset, this.joystickRadius, this.joystickRadius);
        rotateAnim.setInterpolator(new LinearInterpolator());
        rotateAnim.setDuration(0L);
        rotateAnim.setFillAfter(true);
        this.currentRotationRange.startAnimation(rotateAnim);
        rotateAnim = new RotateAnimation(15.0F, 15.0F, this.joystickRadius, this.joystickRadius);
        rotateAnim.setInterpolator(new LinearInterpolator());
        rotateAnim.setDuration(0L);
        rotateAnim.setFillAfter(true);
        this.previousRotationRange.startAnimation(rotateAnim);
    }

    private void updateMagnitudeText() {
        if (!this.turnInPlaceMode) {
            this.magnitudeText.setText((int) (this.normalizedMagnitude * 100.0F) + "%");
            this.magnitudeText.setTranslationX((float) ((double) (this.parentSize / 4.0F) * Math.cos((double) (90.0F + this.contactTheta) * 3.141592653589793D / 180.0D)));
            this.magnitudeText.setTranslationY((float) ((double) (this.parentSize / 4.0F) * Math.sin((double) (90.0F + this.contactTheta) * 3.141592653589793D / 180.0D)));
        }

    }

    private void updateTurnInPlaceRotation() {
        float currentTheta = (this.currentOrientation + 360.0F) % 360.0F;
        float offsetTheta = (this.turnInPlaceStartTheta - currentTheta + 360.0F) % 360.0F;
        offsetTheta = 360.0F - offsetTheta;
        this.magnitudeText.setText(String.valueOf((int) offsetTheta));
        offsetTheta = (float) ((int) (offsetTheta - offsetTheta % 15.0F));
        RotateAnimation rotateAnim = new RotateAnimation(offsetTheta + this.rightTurnOffset, offsetTheta + this.rightTurnOffset, this.joystickRadius, this.joystickRadius);
        rotateAnim.setInterpolator(new LinearInterpolator());
        rotateAnim.setDuration(0L);
        rotateAnim.setFillAfter(true);
        this.currentRotationRange.startAnimation(rotateAnim);
        rotateAnim = new RotateAnimation(offsetTheta + 15.0F, offsetTheta + 15.0F, this.joystickRadius, this.joystickRadius);
        rotateAnim.setInterpolator(new LinearInterpolator());
        rotateAnim.setDuration(0L);
        rotateAnim.setFillAfter(true);
        this.previousRotationRange.startAnimation(rotateAnim);
    }

    private void updateThumbDivet(float x, float y) {
        this.thumbDivet.setTranslationX(-16.5F);
        this.thumbDivet.setTranslationY(-16.5F);
        this.thumbDivet.setRotation(this.contactTheta);
        this.thumbDivet.setTranslationX(x);
        this.thumbDivet.setTranslationY(y);
    }

    private boolean floatCompare(float v1, float v2) {
        return Math.abs(v1 - v2) < 0.001F;
    }

    private boolean inLastContactRange(float x, float y) {
        return Math.sqrt((x - (float) this.contactUpLocation.x - this.joystickRadius) * (x - (float) this.contactUpLocation.x - this.joystickRadius) + (y - (float) this.contactUpLocation.y - this.joystickRadius) * (y - (float) this.contactUpLocation.y - this.joystickRadius)) < 16.5D;
    }

    public void setTopicName(String topicName) {
        this.topicName = topicName;
    }

    public GraphName getDefaultNodeName() {
        return GraphName.of("android_15/virtual_joystick_view");
    }

    public void onStart(ConnectedNode connectedNode) {
        this.publisher = connectedNode.newPublisher(this.topicName, "geometry_msgs/Twist");
        this.currentVelocityCommand = this.publisher.newMessage();
        Subscriber<Odometry> subscriber = connectedNode.newSubscriber("odom", "nav_msgs/Odometry");
        subscriber.addMessageListener(this);
        this.publisherTimer = new Timer();
        this.publisherTimer.schedule(new TimerTask() {
            public void run() {
                if (VirtualJoystickView.this.publishVelocity) {
                    VirtualJoystickView.this.publisher.publish(VirtualJoystickView.this.currentVelocityCommand);
                }

            }
        }, 0L, 80L);
    }

    public void onShutdown(Node node) {
    }

    public void onShutdownComplete(Node node) {
        this.publisherTimer.cancel();
        this.publisherTimer.purge();
    }

    public void onError(Node node, Throwable throwable) {
    }
}
