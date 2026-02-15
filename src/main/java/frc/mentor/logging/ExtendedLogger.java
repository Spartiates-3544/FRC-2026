package frc.mentor.logging;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Map;
import java.util.function.BiConsumer;

import dev.doglog.DogLog;
import edu.wpi.first.units.Measure;
public class ExtendedLogger extends DogLog {
    private static ArrayList<Object> registeredInstances = new ArrayList<Object>();
    private static final Map<Class<?>, BiConsumer<String, Object>> handlers = Map.ofEntries(
        Map.entry(Boolean.class, (path, object) -> log(path, (Boolean)object)),
        Map.entry(Boolean[].class, (path, object) -> log(path, (boolean[])object)),
        Map.entry(Double.class, (path, object) -> log(path, (Double)object)),
        Map.entry(Double[].class, (path, object) -> log(path, (double[])object)),
        Map.entry(Measure.class, (path, object) -> log(path, (Measure<?>)object)),
        Map.entry(Float.class, (path, object) -> log(path, (Float)object)),
        Map.entry(Float[].class, (path, object) -> log(path, (float[])object)),
        Map.entry(Integer.class, (path, object) -> log(path, (Integer)object)),
        Map.entry(Integer[].class, (path, object) -> log(path, (int[])object)),
        Map.entry(Long.class, (path, object) -> log(path, (Long)object)),
        Map.entry(Long[].class, (path, object) -> log(path, (long[])object)),
        Map.entry(String.class, (path, object) -> log(path, (String)object)),
        Map.entry(String[].class, (path, object) -> log(path, (String[])object)),
        Map.entry(Record.class, (path, object) -> log(path, (Record)object))
    );

    /**
     * Enregistrer l'instance d'une classe pour l'utilisation de l'annotation {@literal@}LoggableField. <p>
     * 
     * Exemple:
     *  - Dans le constructeur d'une fonction: ExtendedLogger.registerInstance(this) -> On peut ensuite utiliser {@literal@}LoggableField.
     *
     * @param registeredInstance instance (objet) à enregistrer.
     */
    public static void registerInstance(Object registeredInstance) {
        registeredInstances.add(registeredInstance);
    }

    /**
     * Effectuer toutes les opérations de logging prévues. <p>
     * 
     * Il est important d'appeler cette méthode dans la boucle principale du robot.
     */
    public static void run() {
        logAll();
    }

    /**
     * Logger tous les attributs (fields) annotés. <p>
     * 
     * Appelée automatiquement avec l'utilisation de {@link #run()}.
     */
    public static void logAll() {
        // Pour chaque instance enregistrée...
        for (Object registeredInstance : registeredInstances) {
            // ...et pour chaque attribut de l'instance...
            for (Field field : registeredInstance.getClass().getDeclaredFields()) {
                // ...vérifier s'il possède l'annotation @LoggableField...
                if (field.isAnnotationPresent(LoggableField.class)) {
                    // ...si oui, aller chercher la valeur "path" de l'attribut...
                    String path = field.getAnnotation(LoggableField.class).path();
                    
                    try {
                        // ...et tenter d'aller chercher la valeur de l'attribut, et ce, même s'il est privé...
                        field.setAccessible(true);
                        Object fieldValue = field.get(registeredInstance);
                        // ...ensuite, tenter de logger la valeur de l'attribut...
                        logGeneric(path, fieldValue);
                    } catch (IllegalAccessException e) {
                        // ...en cas d'erreur, afficher un message détaillé.
                        logFault("L'accès à l'attribut " + field.getName() + " est restraint par le système de sécurité Java! (L'attribut est-il privé?)");
                    } catch (IllegalArgumentException e) {
                        // En pratique, ne devrait jamais apparaître.
                        logFault(e.getMessage());
                    }
                }
            }
        }
    }

    /**
     * Logger la valeur d'un objet dont le type n'est pas connu directement.
     * 
     * @param path le chemin sous lequel logger l'objet. (e.g. "ExampleSubsystem/ExampleValue")
     * @param value l'objet à logger
     */
    private static void logGeneric(String path, Object value) {
        if (value == null) {
            // Si la valeur est vide, afficher une erreur...
            logFault("Impossible de logger une valeur null!");
        } else {
            // Sinon, récupérer la méthode "handler"...
            BiConsumer<String, Object> handler = handlers.get(value.getClass());
            
            // ...Si cette méthode est vide et que le type de value ne peut pas être converti...
            if (handler == null) {
                for (Map.Entry<Class<?>, BiConsumer<String, Object>> handlerItem : handlers.entrySet()) {
                    if (handlerItem.getKey().isAssignableFrom(value.getClass())) {
                        handler = handlerItem.getValue();
                        break;
                    }
                }
            }

            // ...Afficher un message d'erreur...
            if (handler == null) {
                logFault("Impossible de logger la valeur de type " + value.getClass().getSimpleName() + " avec @LoggableField ou logGeneric()!");
            } else {
                // ...Sinon, utiliser la méthode "handler" pour appeler la bonne méthode de DogLog.
                handler.accept(path, value);
            }
        }

    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.FIELD)
    public @interface LoggableField {
        String path();
    }

}
