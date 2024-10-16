package frc.robot.filters;

public class BeamBreakFilter {
  // Value used for filter
  private int trueValuer = 0;
  // Exponentially incremented adder
  private int addMultiplier = 1;
  // Exponentially decremented subtractor
  private int subtractMultiplier = 1;
  // Number of consecutive falses
  private int numOfConsecFalses = 0;
  // Threshold for true value
//   private int trueValueThreshold = 10;
  // Maximum true value
  private int maxTrueValue = 10;
  // Threshold for consecutive false reset
  private int falseResetThreshold = 3;

  private int maxFilterVales = 4;

  // Setters
//   public void setTrueValueThreshold(int trueValueThreshold) {
//       this.trueValueThreshold = trueValueThreshold;
//   }
  public void setFalseResetThreshold(int falseResetThreshold) {
      this.falseResetThreshold = falseResetThreshold;
  }

  // filter
  public boolean getFilteredOutput(boolean sensorValue) {
      if (sensorValue) {
          //Exponentially increment true value
          trueValuer += addMultiplier;
          addMultiplier++;
          subtractMultiplier--;
          if (subtractMultiplier > maxFilterVales) {
            subtractMultiplier = maxFilterVales;
          }
          if (addMultiplier > maxFilterVales) {
            addMultiplier = maxFilterVales;
          }
          numOfConsecFalses = 0;
          if (trueValuer > maxTrueValue) {
              trueValuer = maxTrueValue;
          }
      } else {
          // If you have 3 consecutive falses, reset the true value
          if (numOfConsecFalses > falseResetThreshold) {
              trueValuer = 0;
              subtractMultiplier = 1;
              addMultiplier = 1;
          }
          // Exponentially decrement true value
          else {
              trueValuer -= subtractMultiplier;
              numOfConsecFalses++;
              subtractMultiplier++;
              addMultiplier--;
                if (subtractMultiplier > maxFilterVales) {
                    subtractMultiplier = maxFilterVales;
                }
                if (addMultiplier > maxFilterVales) {
                    addMultiplier = maxFilterVales;
                }
          }
      }

      // Check against the threshold
      if (trueValuer > 8) {
          return true;
      } else {
          return false;
      }
  }

}