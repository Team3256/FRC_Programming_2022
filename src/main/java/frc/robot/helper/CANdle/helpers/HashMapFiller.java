package frc.robot.helper.CANdle.helpers;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

public class HashMapFiller {

    /**
     * @param entries Entries defined via entry(), in order to initialize HashMap. (Ex. entry(Key, Value) )
     * @return New LinkedHashMap of entries provided
     */
    public static <K,V> LinkedHashMap<K,V> populateLinkedHashMap(Map.Entry<K, V>... entries){

        LinkedHashMap<K,V> outputHashMap = new LinkedHashMap<>();

        for (Map.Entry<K, V> entry: entries){
            outputHashMap.put(entry.getKey(), entry.getValue());
        }
        return outputHashMap;
    }

    /**
     * @param entries Entries defined via entry(), in order to initialize HashMap. (Ex. entry(Key, Value) )
     * @return New HashMap of entries provided
     */
    public static <K,V> HashMap<K,V> populateHashMap(Map.Entry<K, V>... entries){

        HashMap<K,V> outputHashMap = new HashMap<>();

        for (Map.Entry<K, V> entry: entries){
            outputHashMap.put(entry.getKey(), entry.getValue());
        }
        return outputHashMap;
    }
}
