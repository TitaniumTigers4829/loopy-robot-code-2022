package frc.robot.commands.climb;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

import frc.robot.Constants.ClimbConstants;

public class SaveClimbZeroes {
  
    /**
     * This class is used to save the climb hook values as a text file on the rio.
     * The first line of the text file is for the left hook height, second for the right.
     */
    public SaveClimbZeroes() {}

    public static void writeClimbZeroes(double leftClimbArmZero, double rightClimbArmZero) {
 
        try {
            BufferedWriter fileWriter = new BufferedWriter(
                new FileWriter(ClimbConstants.kClimbTextFilePath)
            );
 
            fileWriter.write(Double.toString(leftClimbArmZero));
            fileWriter.write(Double.toString(rightClimbArmZero));

            fileWriter.close();
        }
        // Catch block to handle if exceptions occurs
        catch (IOException e) {
            // Print the exception on console, use smart dashboard if debugging is needed
            System.out.print(e.getMessage());
        }
    }

    public static double[] readClimbZeroes() {
         
        try {
            BufferedReader fileReader = new BufferedReader(
                new FileReader(ClimbConstants.kClimbTextFilePath)
            );
    
            String leftClimbArmZero = fileReader.readLine();
            String rightClimbArmZero = fileReader.readLine();

            fileReader.close();

            double[] climbZeroes = {Double.parseDouble(leftClimbArmZero), Double.parseDouble(rightClimbArmZero)};

            return climbZeroes;
        }
        // Catch block to handle if exceptions occurs
        catch (IOException e) {
            // Print the exception on console, use smart dashboard if debugging is needed
            System.out.print(e.getMessage());
        }
        // TODO: Get average climb zeroes
        return new double[] {0, 0};
    }

}
