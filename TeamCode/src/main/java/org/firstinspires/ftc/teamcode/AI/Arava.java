package org.firstinspires.ftc.teamcode.AI;
import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import java.io.*;
import java.util.*;
import java.io.IOException;
import java.io.InputStream;
import java.util.Random;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
public class Arava {
    private double[][] hiddenLayerWeights;
    private double[] hiddenLayerBiases;
    private double[] outputLayerWeights;
    private double outputLayerBias;
    private double[] hiddenLayerActivations;

    public Arava() {
        Random random = new Random();
        // Initialize weights and biases for the hidden layer
        hiddenLayerWeights = new double[256][2]; // Adjust dimensions as needed
        for (int i = 0; i < 256; i++) {
            hiddenLayerWeights[i][0] = random.nextDouble();
            hiddenLayerWeights[i][1] = random.nextDouble();
        }
        hiddenLayerBiases = new double[256];
        for (int i = 0; i < 256; i++) {
            hiddenLayerBiases[i] = 0;
        }
        // Initialize weights and bias for the output layer
        outputLayerWeights = new double[256];
        for (int i = 0; i < 256; i++) {
            outputLayerWeights[i] = random.nextDouble();
        }
        outputLayerBias = 0;
    }

    // Forward propagation with ReLU activation function
    public double forwardPropagation(double[] flattenedImage) {
        // Calculate hidden layer activations
        hiddenLayerActivations = new double[256];
        for (int i = 0; i < 256; i++) {
            double z = 0;
            for (int j = 0; j < flattenedImage.length; j++) {
                z += flattenedImage[j] * hiddenLayerWeights[i][j];
            }
            z += hiddenLayerBiases[i];
            hiddenLayerActivations[i] = Math.max(0, z); // ReLU activation
        }
        // Calculate output
        double output = 0;
        for (int i = 0; i < 256; i++) {
            output += hiddenLayerActivations[i] * outputLayerWeights[i];
        }
        output += outputLayerBias;
        return output;
    }

    // Backpropagation with ReLU activation function
    public void backPropagation(double[] flattenedImage, double targetOutput, double learningRate) {
        double output = forwardPropagation(flattenedImage);
        double error = output - targetOutput;
        // Update weights and biases for output layer
        for (int i = 0; i < 256; i++) {
            outputLayerWeights[i] -= learningRate * error * hiddenLayerActivations[i];
        }
        outputLayerBias -= learningRate * error;
        // Calculate deltas for hidden layer neurons
        double[] hiddenLayerDeltas = new double[256];
        for (int i = 0; i < 256; i++) {
            hiddenLayerDeltas[i] = error * outputLayerWeights[i];
        }
        // Update weights and biases for hidden layer
        for (int i = 0; i < 256; i++) {
            double delta = hiddenLayerDeltas[i] * (hiddenLayerActivations[i] > 0 ? 1 : 0); // ReLU derivative
            for (int j = 0; j < flattenedImage.length; j++) {
                hiddenLayerWeights[i][j] -= learningRate * delta * flattenedImage[j];
            }
            hiddenLayerBiases[i] -= learningRate * delta;
        }
    }


    // Training function
    public void train(double[][] inputs, double[] labels, double learningRate, int epochs) {
        for (int epoch = 0; epoch < epochs; epoch++) {
            for (int i = 0; i < inputs.length; i++) {
                backPropagation(inputs[i], labels[i], learningRate);
            }
        }
    }


    // Process image using OpenCV
    public double[] processImage(String fileName) throws IOException {
        // Load OpenCV library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Read the image using OpenCV
        Mat image = Imgcodecs.imread(fileName);

        // Convert the image to grayscale
        Mat grayImage = new Mat();
        Imgproc.cvtColor(image, grayImage, Imgproc.COLOR_BGR2GRAY);

        // Resize the image if needed (assuming the input image is already the desired size)
        // Mat resizedImage = new Mat();
        // Imgproc.resize(grayImage, resizedImage, new Size(desiredWidth, desiredHeight));

        // Convert the image to a double array
        double[] flattenedImage = new double[grayImage.rows() * grayImage.cols()];
        int index = 0;
        for (int y = 0; y < grayImage.rows(); y++) {
            for (int x = 0; x < grayImage.cols(); x++) {
                flattenedImage[index++] = grayImage.get(y, x)[0] / 255.0; // Normalize pixel value to range [0, 1]
            }
        }

        // Return the processed image
        return flattenedImage;
    }

    // Save the model to a file
    public void saveModel(String filePath) throws IOException {
        ObjectOutputStream outputStream = new ObjectOutputStream(new FileOutputStream(filePath));
        outputStream.writeObject(hiddenLayerWeights);
        outputStream.writeObject(hiddenLayerBiases);
        outputStream.writeObject(outputLayerWeights);
        outputStream.writeObject(outputLayerBias);
        outputStream.close();
    }

    // Load the model from a file
    public void loadModel(String filePath) throws IOException, ClassNotFoundException {
        ObjectInputStream inputStream = new ObjectInputStream(new FileInputStream(filePath));
        hiddenLayerWeights = (double[][]) inputStream.readObject();
        hiddenLayerBiases = (double[]) inputStream.readObject();
        outputLayerWeights = (double[]) inputStream.readObject();
        outputLayerBias = (double) inputStream.readObject();
        inputStream.close();
    }

    // Calculate accuracy
    public static double calculateAccuracy(double predictedValue, double actualValue) {
        return Math.abs(predictedValue - actualValue) / actualValue;
    }

    public static void main(String[] args) throws IOException {
        Arava ai = new Arava();
        System.setProperty("java.library.path", "path/to/opencv/native/libraries");
        // Load the library
        System.loadLibrary("opencv_java470");

        // Example image file names
        String[] imageFileNames = {"blue_center.jpg", "blue_center2.jpg", "blue_center3.jpg", "blue_left.jpg", "blue_left2.jpg", "blue_left3.jpg"
        , "blue_right.jpg", "blue_right2.jpg", "blue_right3.jpg", "blue_right4.jpg", "blue_right5.jpg", "red_center.jpg", "red_center2.jpg", "red_center3.jpg"
        , "red_left.jpg", "red_left2.jpg", "red_left3.jpg", "red_right.jpg", "red_right2.jpg", "red_right3.jpg"};

        // Process each image and add its flattened representation to the inputs array
        double[][] inputs = new double[imageFileNames.length][];
        for (int i = 0; i < imageFileNames.length; i++) {
            String fileName = imageFileNames[i];
            inputs[i] = ai.processImage(fileName);
        }

       // Example labels representing the position of the object in the image
        double[] labels = {1.0, 2.0, 3.0}; // Assuming 1.0 for right, 2.0 for center, 3.0 for left

        double learningRate = 0.01;
        int epochs = 1000;
        // Train the network
        ai.train(inputs, labels, learningRate, epochs);

        // Test the network with a new image
        String testImageFileName = "blue_right.jpg";
        double[] testImage = ai.processImage(testImageFileName);
        double output = ai.forwardPropagation(testImage);
        System.out.println("Predicted output: " + output);
        System.out.println("Predicted accuracy: " + (100 - calculateAccuracy(output, 2.0)));
    }
}
