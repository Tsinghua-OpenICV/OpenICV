# Message Structure

```plain-text
+---------------+---------------+
|       0       |       1       |
|0|1|2|3|4|5|6|7|0|1|2|3|4|5|6|7|
+---------------+---------------+
|V=2|I|C|A=2|P=2|       S       |
+---------------+---------------+
|         [Index]/Data          |
|         [Index]/Data          |
|             Data              |
|              ...              |
+---------------+---------------+
```

- V: Protocol Version
- I: Whether the package is about connection initialization
- C: Whether the package is the confirm of connection
- A: The length (in Byte) of index for sliced data
- P: Preserved
- S: Sequence number indicating the order of data from certain output port, starting from 0.
- Index: The index indicating the order of slices of this data. The length of this field is `A` Bytes. (Big-endian)

# Remote Data Transmission Procedure
- The output port owner broadcast the available output ports name (Through UDP port 82)
- All receivers add the name to hash table, and then every ports is mapped to a UDP port
- The output port multicast data through given port, receiver will read data from listened port
