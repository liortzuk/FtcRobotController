package org.firstinspires.ftc.teamcode.AI;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import static java.lang.Math.abs;

import android.graphics.Camera;
import android.hardware.camera2.CameraDevice;

import org.firstinspires.ftc.teamcode.DriveTrain.DriveTrain;
import org.firstinspires.ftc.teamcode.Elevator.Elevator;
import com.acmerobotics.dashboard.FtcDashboard;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import java.io.*;
import java.util.Random;


// Java Program to take a Snapshot from System Camera
// using OpenCV

// Importing openCV modules
import org.opencv.core.Core;
import org.opencv.core.Mat;
// This class is responsible for taking screenshot
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;


public class Arava implements Serializable {

    HardwareMap hardwareMap;
    private double[][] hiddenLayerWeights;
    private double[] hiddenLayerBiases;
    private double[] outputLayerWeights;
    private double outputLayerBias;
    private double[] hiddenLayerActivations; // Added this field

    private double bestAccuracy = Double.MIN_VALUE;
    private double[][] bestHiddenLayerWeights;
    private double[] bestHiddenLayerBiases;
    private double[] bestOutputLayerWeights;
    private double bestOutputLayerBias;

    private double[][] training_data;
    private double[] training_label;

    private double[][] testing_data;
    private double[] testing_labels;

    private String[] imageFileNames = {
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\blue_center.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\blue_center2.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\blue_center3.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\blue_left.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\blue_left2.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\blue_left3.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\blue_right.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\blue_right2.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\blue_right3.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\blue_right4.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\blue_right5.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\red_center.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\red_center2.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\red_center3.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\red_left.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\red_left2.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\red_left3.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\red_right.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\red_right2.jpeg",
            "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\red_right3.jpeg"
    };
    public Arava() {
        Random random = new Random();
        hiddenLayerWeights = new double[256][2];
        for (int i = 0; i < 256; i++) {
            hiddenLayerWeights[i][0] = random.nextGaussian() * Math.sqrt(2.0 / 256);
            hiddenLayerWeights[i][1] = random.nextGaussian() * Math.sqrt(2.0 / 256);
        }
        hiddenLayerBiases = new double[256];
        for (int i = 0; i < 256; i++) {
            hiddenLayerBiases[i] = 0;
        }
        outputLayerWeights = new double[256];
        for (int i = 0; i < 256; i++) {
            outputLayerWeights[i] = random.nextGaussian() * Math.sqrt(2.0 / 256);
        }
        outputLayerBias = 0;
    }

    public double forwardPropagation(double[] flattenedImage) {
        hiddenLayerActivations = new double[256];
        for (int i = 0; i < 256; i++) {
            double z = 0;
            for (int j = 0; j < flattenedImage.length; j++) {
                if (j < hiddenLayerWeights[i].length) {
                    z += flattenedImage[j] * hiddenLayerWeights[i][j];
                }
            }
            z += hiddenLayerBiases[i];
            hiddenLayerActivations[i] = Math.max(0, z);
        }
        double output = 0;
        for (int i = 0; i < 256; i++) {
            output += hiddenLayerActivations[i] * outputLayerWeights[i];
        }
        output += outputLayerBias;
        return output;
    }

    public void backPropagation(double[] flattenedImage, double targetOutput, double learningRate) {
        double output = forwardPropagation(flattenedImage);
        double error = output - targetOutput;
        for (int i = 0; i < 256; i++) {
            if (i < hiddenLayerActivations.length) {
                outputLayerWeights[i] -= learningRate * error * hiddenLayerActivations[i];
            }
        }
        outputLayerBias -= learningRate * error;
        double[] hiddenLayerDeltas = new double[256];
        for (int i = 0; i < 256; i++) {
            hiddenLayerDeltas[i] = error * outputLayerWeights[i];
        }
        for (int i = 0; i < 256; i++) {
            if (i < hiddenLayerActivations.length) {
                double delta = hiddenLayerDeltas[i] * (hiddenLayerActivations[i] > 0 ? 1 : 0);
                for (int j = 0; j < flattenedImage.length; j++) {
                    if (j < hiddenLayerWeights[i].length) {
                        hiddenLayerWeights[i][j] -= learningRate * delta * flattenedImage[j];
                    }
                }
                hiddenLayerBiases[i] -= learningRate * delta;
            }
        }
    }

    //training loop (gradiant decent)
    public void train(double[][] inputs, double[] labels, double learningRate, int epochs) {
        int numSamples = inputs.length;
        for (int epoch = 0; epoch < epochs; epoch++) {
            //shuffle(inputs, labels);
            training_and_testing_data(inputs, labels);

            for (int i = 0; i < numSamples; i++) {
                backPropagation(training_data[i], training_label[i], learningRate);
                double output = forwardPropagation(training_data[i]);
                double totalAccuracy = 0;
                for (int j = 0; j <= i; j++) {
                    double accuracy = calculateAccuracy(output, training_label[j]);
                    totalAccuracy += accuracy;
                }
                double averageAccuracy = totalAccuracy / (i + 1);
                if (averageAccuracy > bestAccuracy) {
                    bestAccuracy = averageAccuracy;
                    bestHiddenLayerWeights = hiddenLayerWeights.clone();
                    bestHiddenLayerBiases = hiddenLayerBiases.clone();
                    bestOutputLayerWeights = outputLayerWeights.clone();
                    bestOutputLayerBias = outputLayerBias;
                }
                System.out.println("Epoch " + (epoch + 1) + " - Image: " + imageFileNames[i] + " - Predicted Output: " + output + " - Accuracy: " + calculateAccuracy(output, labels[i]) + "%");
            }
        }
        System.out.println("best accuracy: " + bestAccuracy);
        saveModel("C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\best_model.ser", bestHiddenLayerWeights, bestHiddenLayerBiases, bestOutputLayerWeights, bestOutputLayerBias);
    }

    public void training_and_testing_data(double[][] inputs, double[] labels){
        training_data = new double[inputs.length][inputs.length];
        training_label = new double[labels.length];
        int count = 0;

        for (int i = 0; i < inputs.length; i++){
            training_data[i] = inputs[i];
            training_label[i] = labels[i];
            count ++;
        }
    }

    private void shuffle(double[][] inputs, double[] labels) {
        Random rnd = new Random();
        for (int i = inputs.length - 1; i > 0; i--) {
            int index = rnd.nextInt(i + 1);

            // Swap inputs
            double[] tempInput = inputs[index];
            inputs[index] = inputs[i];
            inputs[i] = tempInput;

            // Swap labels
            double tempLabel = labels[index];
            labels[index] = labels[i];
            labels[i] = tempLabel;
        }
    }


    public double[] processImage(String fileName) throws IOException {
        Mat image = Imgcodecs.imread(fileName);
        Mat grayImage = new Mat();
        Imgproc.cvtColor(image, grayImage, Imgproc.COLOR_BGR2GRAY);
        double[] flattenedImage = new double[grayImage.rows() * grayImage.cols()];
        int index = 0;
        for (int y = 0; y < grayImage.rows(); y++) {
            for (int x = 0; x < grayImage.cols(); x++) {
                flattenedImage[index++] = grayImage.get(y, x)[0] / 255.0;
            }
        }
        return flattenedImage;
    }

    public void loadModel(String filePath) {
        try {
            ObjectInputStream inputStream = new ObjectInputStream(new FileInputStream(filePath));
            hiddenLayerWeights = (double[][]) inputStream.readObject();
            hiddenLayerBiases = (double[]) inputStream.readObject();
            outputLayerWeights = (double[]) inputStream.readObject();
            outputLayerBias = (double) inputStream.readObject();
            inputStream.close();
        } catch (IOException | ClassNotFoundException e) {
            System.err.println("Error loading model: " + e.getMessage());
        }
    }

    public void saveModel(String filePath, double[][] hiddenLayerWeights, double[] hiddenLayerBiases, double[] outputLayerWeights, double outputLayerBias) {
        try {
            ObjectOutputStream outputStream = new ObjectOutputStream(new FileOutputStream(filePath));
            outputStream.writeObject(hiddenLayerWeights);
            outputStream.writeObject(hiddenLayerBiases);
            outputStream.writeObject(outputLayerWeights);
            outputStream.writeObject(outputLayerBias);
            outputStream.close();
        } catch (IOException e) {
            System.err.println("Error saving model: " + e.getMessage());
        }
    }

    public static double calculateAccuracy(double predictedValue, double actualValue) {
        double absoluteError = Math.abs(predictedValue - actualValue);
        return (1 - absoluteError / actualValue) * 100;
    }

    public void checkImageFilesExist(String[] imageFileNames) {
        for (String fileName : imageFileNames) {
            File file = new File(fileName);
            if (file.exists() && file.isFile()) {
                System.out.println("Image file found: " + fileName);
            } else {
                System.out.println("Image file not found: " + fileName);
            }
        }
    }

    // Method to capture frame from the camera
    Mat mat = new Mat();
    public Mat inputFrame(Mat input) {
        Imgproc.cvtColor(input, mat,  Imgproc.COLOR_BGR2GRAY);
        return mat;
    }

    // Modify Run_the_AI method to use the captured frame for prediction
    public int Run_the_AI() {
        OpenCvCamera camera;
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cum"), hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        camera.startStreaming(1920, 1080);
        examplePipeline examplePipeline = new examplePipeline();
        camera.setPipeline(examplePipeline);

        Mat frame = examplePipeline.mat;

        if (frame != null) {
            Mat grayFrame = new Mat();
            Imgproc.cvtColor(frame, grayFrame, Imgproc.COLOR_BGR2GRAY);
            double[] flattenedImage = new double[grayFrame.rows() * grayFrame.cols()];
            int index = 0;
            for (int y = 0; y < grayFrame.rows(); y++) {
                for (int x = 0; x < grayFrame.cols(); x++) {
                    flattenedImage[index++] = grayFrame.get(y, x)[0] / 255.0;
                }
            }
            double output = forwardPropagation(flattenedImage);

            if (output <= 1.5) {
                output = 1.0;
            } else if (output > 1.5 && output < 2.5) {
                output = 2.0;
            } else if (output >= 2.5) {
                output = 3.0;
            }

            System.out.println("Predicted output: " + output);
            return (int) output;
        } else {
            return 0; // Return -1 to indicate failure
        }
    }



    public static void main(String[] args) throws IOException {
        Arava ai = new Arava();

        // Check if image files exist
        ai.checkImageFilesExist(ai.imageFileNames);

        // Process images using OpenCV
        double[][] inputs = new double[ai.imageFileNames.length][];
        for (int i = 0; i < ai.imageFileNames.length; i++) {
            String fileName = ai.imageFileNames[i];
            inputs[i] = ai.processImage(fileName);
        }

        // Labels
        double[] labels = {2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 1.0, 1.0, 1.0};

        // Load the model if it exists
        String modelFilePath = "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\best_model.ser";
        File modelFile = new File(modelFilePath);
        if (modelFile.exists()) {
            ai.loadModel(modelFilePath);
            System.out.println("Model loaded successfully.");
        }

        // Training
        double learningRate = 0.01;
        int epochs = 100;
        // ai.train(inputs, labels, learningRate, epochs);

        // Test the network with a new image
        String testImageFileName = "C:\\Users\\Saar Tzuk\\StudioProjects\\FtcRobotController\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode\\AI\\Images\\blue_center3.jpeg";
        double[] testImage = ai.processImage(testImageFileName);
        double output = ai.forwardPropagation(testImage);

        if (output > 0.5 && output < 1.5) {
            output = 1.0;
        } else if (output > 1.5 && output < 2.5) {
            output = 2.0;
        } else if (output > 2.5) {
            output = 3.0;
        }

        System.out.println("Predicted output: " + output);
        System.out.println("Predicted accuracy: " + ai.calculateAccuracy(output, 2.0));

    }
}

