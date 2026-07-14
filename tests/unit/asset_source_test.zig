const std = @import("std");
const source = @import("gravity").assets.source;
test "canonical source JSON has stable source IDs and decimal reals" {
    try source.validate("{\"assets\":[{\"source_id\":1,\"kind\":\"Sphere\",\"radius\":\"1.25\"},{\"source_id\":2,\"kind\":\"Material\",\"friction\":\"0\"}]}", std.testing.allocator);
}
test "source JSON rejects floats duplicate ids and invalid decimals" {
    try std.testing.expectError(error.NonCanonicalNumber, source.validate("{\"assets\":[{\"source_id\":1,\"kind\":\"Sphere\",\"radius\":1.2}]}", std.testing.allocator));
    try std.testing.expectError(error.DuplicateSourceId, source.validate("{\"assets\":[{\"source_id\":2,\"kind\":\"Sphere\"},{\"source_id\":1,\"kind\":\"Box\"}]}", std.testing.allocator));
    try std.testing.expectError(error.InvalidDecimal, source.validate("{\"assets\":[{\"source_id\":1,\"kind\":\"Sphere\",\"radius\":\"1e2\"}]}", std.testing.allocator));
}
