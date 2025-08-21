package ch.chris_hoffmann.ors.routing.graphhopper.extensions.weighting;

import com.graphhopper.routing.ev.*;
import com.graphhopper.routing.util.EncodingManager;
import com.graphhopper.routing.util.FlagEncoder;
import com.graphhopper.routing.weighting.AbstractWeighting;
import com.graphhopper.routing.weighting.TurnCostProvider;
import com.graphhopper.util.PMap;
import com.graphhopper.util.EdgeIteratorState;
import com.graphhopper.storage.GraphHopperStorage;
import com.graphhopper.routing.weighting.Weighting;
import org.heigit.ors.routing.graphhopper.extensions.ORSWeightingFactory;
import org.heigit.ors.routing.graphhopper.extensions.flagencoders.FlagEncoderKeys;

import java.util.logging.Logger;

import static com.graphhopper.routing.util.EncodingManager.getKey;

public class CurvinessWeighting extends AbstractWeighting {
    private final DecimalEncodedValue speedEnc;           // vehicle average speed (km/h)
    private final DecimalEncodedValue maxSpeedEnc;        // signed/decimal max speed if present (km/h)
    private final EnumEncodedValue<RoadClass> roadClass;  // motorway/primary/... classification
    private final DecimalEncodedValue curvinessEnc;       // your custom per-edge metric

    private static final org.apache.log4j.Logger LOGGER = org.apache.log4j.Logger.getLogger(CurvinessWeighting.class.getName());

    // tunables
    private final double alpha;   // strength for curviness preference
    private final double beta;    // strength for speed-limit band preference
    private final double highwayPenalty; // multiplicative penalty for motorways


    public CurvinessWeighting(GraphHopperStorage gh, FlagEncoder encoder, PMap hints) {
        super(encoder);
        var em = gh.getEncodingManager();

        // average speed for this encoder; GraphHopper composes the key per-vehicle
        this.speedEnc     = em.getDecimalEncodedValue(EncodingManager.getKey(encoder, "average_speed"));
        // max speed value (if available from OSM; may be NaN or 0 when unknown)
        this.maxSpeedEnc  = em.hasEncodedValue(MaxSpeed.KEY)
                ? em.getDecimalEncodedValue(MaxSpeed.KEY)
                : null;

        this.roadClass    = em.getEnumEncodedValue(RoadClass.KEY, RoadClass.class);
        this.curvinessEnc = em.getDecimalEncodedValue("curviness"); // you registered this

        // read tunables from profile hints (with defaults)
        this.alpha = hints.getDouble("curvy.alpha", 0.5);             // 0.0..2.0 sensible
        this.beta  = hints.getDouble("curvy.beta", 0.6);              // 0.0..1.0
        this.highwayPenalty = hints.getDouble("curvy.highway_penalty", 10.0); // 10x weight
    }

    @Override
    public double getMinWeight(double v) {
        return 0;
    }

    @Override public double calcEdgeWeight(EdgeIteratorState edge, boolean reverse) {
        // distance in meters
        final double distM = edge.getDistance();

        // km/h from average speed EV
        final double speedKmh = edge.get(speedEnc);
        if (speedKmh <= 0) return Double.POSITIVE_INFINITY;

        // base travel time in seconds
        final double tSec = distM / (speedKmh * (1000d/3600d));

        // curviness: dimensionless; 0 = straight, higher = curvier
        final double curvy = edge.get(curvinessEnc);

        // prefer 70–100 km/h limits (triangle preference peaking ~85 km/h)
        double limit = (maxSpeedEnc != null) ? edge.get(maxSpeedEnc) : Double.NaN;
        if (Double.isNaN(limit) || limit <= 0) limit = speedKmh; // fallback
        final double speedPref = bandPreference(limit, 70, 100, 85); // 0..1

        // penalize motorways (avoid highways)
        final boolean isMotorway = edge.get(roadClass) == RoadClass.MOTORWAY;

        // combine: lower weight is better
        double weight = tSec;
        weight /= (1.0 + alpha * curvy);           // reward curviness
        weight /= (1.0 + beta  * speedPref);       // reward desired speed band
        if (isMotorway) weight *= highwayPenalty;  // avoid highways

        return weight;
    }

    // Weighting also needs millis; use base time (without preferences) for ETA correctness
    @Override public long calcEdgeMillis(EdgeIteratorState edge, boolean reverse) {
        final double speedKmh = edge.get(speedEnc);
        if (speedKmh <= 0) return Long.MAX_VALUE;
        final double tSec = edge.getDistance() / (speedKmh * (1000d/3600d));
        return (long) (tSec * 1000);
    }

    @Override public String getName() { return "curviness"; }

    private static double bandPreference(double v, double min, double max, double peak) {
        if (v <= min || v >= max) return 0.0;
        // linear rise to peak, then linear fall
        return (v <= peak) ? (v - min) / (peak - min) : (max - v) / (max - peak);
    }
}
