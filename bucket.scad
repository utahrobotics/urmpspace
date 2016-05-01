conversion = 254;

bucket_width = 28.25 * conversion;
bucket_length = 12.5 * conversion;
bucket_height = 6.5 * conversion;
bucket_diagonal = 12.5 * conversion;

bucket_thickness = .138 * conversion;

angle = acos((pow(bucket_length, 2) + pow(bucket_height, 2) - pow(bucket_diagonal, 2)) / (2 * bucket_length * bucket_height));

echo(angle);

bucket();

module bucket() {
	difference() {
		hull() {
			translate([-bucket_width/2, 0, 0])
			bucket_side();

			translate([bucket_width/2 - bucket_thickness, 0, 0])
			bucket_side();
		}

		translate([0, bucket_thickness / sin(angle), bucket_thickness])
		hull() {
			translate([-bucket_width/2 + bucket_thickness, 0, 0])
			bucket_side();

			translate([bucket_width/2 - bucket_thickness * 2, 0, 0])
			bucket_side();
		}
	}
}

module bucket_side() {
	rotate([0, -90, 0])
	linear_extrude(height = bucket_thickness) {
		polygon(points = [
				[0, 0],
				[0, bucket_length],
				[bucket_height * sin(angle), bucket_height * cos(angle)]],

				paths = [[0, 1, 2]]);
	}
}

module bucket_shape() {
	polyhedron(points = [
			[0, 0, 0],
			[bucket_width, 0, 0],
			[bucket_width, bucket_length, 0],
			[0, bucket_length, 0],
			[0, bucket_height * cos(angle), bucket_height * sin(angle)],
			[bucket_width, bucket_height * cos(angle), bucket_height * sin(angle)]],
	
			faces = [
			[0, 1, 2, 3], [0, 3, 4], [1, 2, 5], [0, 1, 5, 4], [2, 3, 4, 5]]);
}
