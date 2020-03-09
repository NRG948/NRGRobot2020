package frc.robot.utilities;

import java.util.ArrayList;

/**
 * An class implementing the subject in the Observer Pattern.
 * 
 * @see https://en.wikipedia.org/wiki/Observer_pattern
 * 
 * @param <T> The type of the subject.
 */
public class Subject<T> {
    public ArrayList<Observer<T>> observers = new ArrayList<Observer<T>>();

    public void addObserver(Observer<T> observer) {
        observers.add(observer);
    }

    public void removeObserver(Observer<T> observer) {
        observers.remove(observer);
    }

    public void removeAllObservers() {
        observers = new ArrayList<Observer<T>>();
    }

    public void notify(T subject) {
        observers.forEach(o -> o.update(subject));
    }
}
