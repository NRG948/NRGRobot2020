package frc.robot.utilities;

/**
 * An interface implemented by the observer of a subject in the Observer Pattern.
 * 
 * @see https://en.wikipedia.org/wiki/Observer_pattern
 * 
 * @param <T> The type of the subject.
 */
public interface Observer<T> {
    public void update(T subject);
}
