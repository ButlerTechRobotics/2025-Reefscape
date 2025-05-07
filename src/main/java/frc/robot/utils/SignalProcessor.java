// Copyright (c) 2025 FRC 325/144 & 5712
// https://hemlock5712.github.io/Swerve-Setup/home.html
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.utils;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Provides multithreaded hardware signal processing capabilities integrated with AdvantageKit.
 * Allows for asynchronous processing of sensor data without blocking the main robot loop.
 */
public class SignalProcessor {
  private static final ExecutorService executor = Executors.newFixedThreadPool(3);

  /**
   * Process a signal asynchronously and log the result with AdvantageKit.
   *
   * @param <T> The type of data being processed
   * @param supplier Function that provides the raw sensor data
   * @param processor Function that processes the raw data
   * @param logKey The key to use when logging the processed result
   * @return A Future representing the pending result of the processing
   */
  public static <T> Future<?> processAsync(
      Supplier<T> supplier, Consumer<T> processor, String logKey) {
    return executor.submit(
        () -> {
          try {
            T data = supplier.get();
            processor.accept(data);
            Logger.recordOutput(logKey + "/Processed", true);
          } catch (Exception e) {
            Logger.recordOutput(logKey + "/Error", e.getMessage());
            Logger.recordOutput(logKey + "/Processed", false);
          }
        });
  }

  /**
   * Process a signal with a transformation function asynchronously and log the result.
   *
   * @param <T> The type of input data
   * @param <R> The type of output data
   * @param supplier Function that provides the raw sensor data
   * @param transformer Function that transforms the raw data to processed data
   * @param logKey The key to use when logging the processed result
   * @return A Future containing the processed result
   */
  public static <T, R> Future<R> processTransformAsync(
      Supplier<T> supplier, java.util.function.Function<T, R> transformer, String logKey) {
    return executor.submit(
        () -> {
          try {
            T data = supplier.get();
            R result = transformer.apply(data);
            Logger.recordOutput(logKey + "/ProcessedValue", result.toString());
            Logger.recordOutput(logKey + "/Processed", true);
            return result;
          } catch (Exception e) {
            Logger.recordOutput(logKey + "/Error", e.getMessage());
            Logger.recordOutput(logKey + "/Processed", false);
            throw e;
          }
        });
  }

  /**
   * Filters a signal using a low-pass filter algorithm to reduce noise.
   *
   * @param currentValue The current sensor reading
   * @param previousFilteredValue The previous filtered value
   * @param alpha The filter coefficient (0.0 to 1.0, lower = more filtering)
   * @return The filtered value
   */
  public static double lowPassFilter(
      double currentValue, double previousFilteredValue, double alpha) {
    return alpha * currentValue + (1 - alpha) * previousFilteredValue;
  }

  /**
   * Shuts down the executor service when the robot is disabled or program is ending. Should be
   * called during robot disable or when the program is shutting down.
   */
  public static void shutdown() {
    executor.shutdown();
  }
}
