#autopilot/route-manager/signals/edited

var TYPE_MIX  = 0;# plan has both mission and RTB
var TYPE_RTB  = 1;# return to base plan
var TYPE_MISS = 2;# mission plan

var TRUE  = 1;
var FALSE = 0;

var Polygon = {
	#
	# Class methods and variables
	#
	primary: nil,
	editing: nil,
	editRTB: nil,
	editMiss: nil,
	polys: {},
	_activating: FALSE,
	flyRTB: nil,
	flyMiss: nil,

	setupJAPolygons: func {
		Polygon._setupListeners();
		me.multi = getprop("ja37/supported/multiple-flightplans");
		if (me.multi == TRUE) {
			var poly1 = Polygon.new("1", TYPE_MISS, TRUE);
			Polygon.polys["1"] = poly1;
			for (var i = 2; i<=4; i+=1) {
				var poly = Polygon.new(""~i, TYPE_MISS);
				Polygon.polys[""~i] = poly;
			}
			var polyA = Polygon.new("A", TYPE_RTB);
			Polygon.polys["A"]   = polyA;

			var polyB = Polygon.new("B", TYPE_RTB);
			Polygon.polys["B"]   = polyB;

			Polygon.primary      = poly1;
			Polygon.editing      = poly1;
			Polygon.editRTB      = polyA;
			Polygon.editMiss     = poly1;
			Polygon.flyRTB       = polyA;
			Polygon.flyMiss      = poly1;
		} else {
			var poly1 = Polygon.new("1", TYPE_MIX, TRUE);
			Polygon.primary      = poly1;
			Polygon.editing      = poly1;
			Polygon.flyRTB       = poly1;
			Polygon.flyMiss      = poly1;
			Polygon.editRTB      = poly1;
			Polygon.editMiss     = poly1;
		}
		print("finished plan Init");
	},

	# TODO make AJ polys

	getLandingBase: func {
		return Polygon.flyRTB.getBase();
	},

	isLandingBaseRunwayActive: func {
		if (Polygon.flyRTB == Polygon.primary and Polygon.isPrimaryActive() == TRUE and Polygon.primary.plan.destination_runway != nil and Polygon.primary.getSize()-1==Polygon.primary.plan.current) {
			return TRUE;
		}
	},

	startPrimary: func {
		fgcommand("activate-flightplan", props.Node.new({"activate": 1}));
	},

	activateLandingBase: func {
		me.base = Polygon.getLandingBase();
		Polygon.flyRTB.activate();
		if (me.base != nil) {
			Polygon.primary.current = Polygon.primary.getSize()-1;
			if(Polygon.primary.forceRunway()) {
				Polygon.startPrimary();
			}
		} else {
			Polygon.deactivate();
			return FALSE;
		}
	},

	deactivate: func {
		fgcommand("activate-flightplan", props.Node.new({"activate": 0}));
		#if (Polygon.isPrimaryActive() == TRUE) {
		#	Polygon.primary.plan.cleanPlan();#careful here, if called twice it will clean plan.
		#}
	},

	isPrimaryActive: func {
		return getprop("autopilot/route-manager/active");
	},

	_setupListeners: func {
		setlistener("autopilot/route-manager/signals/flightplan-changed", func Polygon._planExchange);
	},

	_planExchange: func {
		print("plan exhanged");
		if (Polygon._activating == TRUE) {
			Polygon._activating = FALSE;
			print("..it was planned");
		} else {
			# New plan was loaded in route-manager
			var poly = Polygon.primary;
			poly.plan = flightplan();
			poly.plan.id = plan.name;
			print("..it was unexpected");
		}
	},

	_finishedPrimary: func {
		#called from RouteManagerDelegate
		print("plan finished");
		if (Polygon._activating == FALSE) {
			if (Polygon.primary.type == TYPE_MISS) {
				Polygon.flyRTB.activate();
				Polygon.startPrimary();
				print("..starting "~Polygon.flyRTB.name);
			} elsif (Polygon.primary.type == TYPE_RTB) {
				Polygon.startPrimary();
				Polygon.selectDestinationOnPrimary();
				print("..restarted "~Polygon.flyMiss.name);
			}
		} else {
			print("..for real");
		}
		# TODO Handle that finish is called just from activating another.
	},

	selectDestinationOnPrimary: func {
		Polygon.primary.plan.current = Polygon.primary.plan.getPlanSize()-1;
		return Polygon.primary.plan.getPlanSize() != 0;
	},

	#
	# Instance methods and variables
	#
	new: func (name, type, default = 0) {
		var newPoly = { parents : [Polygon]};
		if (default == 1) {
			newPoly.plan = flightplan();
		} else {
			newPoly.plan = flightplan().clone();
			newPoly.plan.cleanPlan();
		}
		if (type == TYPE_RTB) {
			newPoly.plan.departure = nil;
		}
		#newPoly.plan.destination = nil;   error in FG
		newPoly.plan.id = name;
		newPoly.name = name;
		newPoly.type = type;
		return newPoly;
	},

	getPolygon: func {
		me.numbers = me.plan.getPlanSize();
		me.polygon = [];
		for (var i = 0; i < me.numbers; i+=1) {
			append(me.polygon, me.plan.getWP(i));
		}
		return me.polygon;
	},

	getSteerpoint: func {
		if (me.plan.current == me.getSize()-1 and me.plan.destination_runway != nil and me.plan.destination != nil) {
			return [me.plan.destination_runway, me.plan.destination];
		}
		if (me.plan.current == me.getSize()-1 and me.plan.destination != nil) {
			return [me.plan.destination];
		}
		if (me.plan.current == 0 and me.plan.departure_runway != nil and me.plan.departure != nil) {
			return [me.plan.departure_runway, me.plan.departure];
		}
		if (me.plan.current == 0 and me.plan.departure != nil) {
			return [me.plan.departure];
		}
		return [me.plan.currentWP()];
	},

	getIndex: func {
		return me.plan.current;
	},

	getLeg: func {
		return me.plan.currentWP();
	},

	activate: func {
		Polygon._activating = TRUE;
		Polygon.primary = me;
		me.plan.activate();
	},

	isPrimary: func {
		return Polygon.primary == me;
		if (me.multi == TRUE) {
			return me.plan.active;
		} else {
			return TRUE;
		}
	},

	getSize: func {
		return me.plan.getPlanSize();
	},

	cycle: func {
		# Cycle waypoints
		if (me.plan.current = me.getSize()-1) {
			me.plan.current = 0;
		} else {
			me.plan.current = me.plan.current +1;
		}
	},

	getBase: func {
		# Will return destination or last waypoint (if its a airport).
		if (me.getSize() > 0) {
			if(me.plan.destination != nil) {
				return me.plan.destination;
			} else {
				me.base = me.plan.getWP(me.getSize()-1);
				if (ghosttype(me.base) == "airport") {
					return me.base;
				}
			}
		}
		return nil;
	},

	forceRunway: func {
		# will only force a runway onto a destination.
		me.base = me.plan.destination;
		me.baseRwy = me.plan.destination_runway;
		if (me.baseRwy == nil and me.base != nil) {
			me.runways = me.base.runways;
	        me.runwaysVector = [];
	        foreach (var runwayKey ; keys(me.runways)) {
	            append(me.runwaysVector, me.runways[runwayKey]);
	        }
	        if (size(me.runwaysVector)!=0) {
	        	me.plan.destination_runway = me.runwaysVector[0];
	        	return TRUE;
	        }
		} elsif (me.baseRwy != nil) {
			return TRUE;
		}
		return FALSE;
	},

	cycleDestinationRunway: func {
		# cycle through destination runways.
		me.base = me.plan.destination;
		me.baseRwy = me.plan.destination_runway;
		me.shouldStart = me.isPrimary() == TRUE and Polygon.isPrimaryActive() == TRUE;
		if (me.base != nil) {
			me.runways = me.base.runways;
	        me.runwaysVector = [];
	        foreach (var runwayKey ; keys(me.runways)) {
	            append(me.runwaysVector, me.runways[runwayKey]);
	        }
	        if (size(me.runwaysVector)==0) {
	        	return FALSE;
	        }
	        me.currRWY = -1;
	        if (me.baseRwy != nil) {
		        for (var i = 0; i<size(me.runwaysVector);i+=1) {
		            if (me.runwaysVector[i].id == me.baseRwy.id) {
		                me.currRWY = i;
		                break;
		            }
		        }
		    }
	        me.currRWY += 1;
	        if (me.currRWY >= size(me.runwaysVector)) {
	            me.currRWY = 0;
	        }
	        me.plan.destination_runway = me.runwaysVector[me.currRWY];	 
	        if (me.shouldStart == TRUE) {
	        	Polygon.startPrimary();
	        	me.plan.current = me.getSize()-1;
	        }
	        return TRUE;
		} else {
			return FALSE;
		}
	},
};

var poly_start = func {
	removelistener(lsnr);
	Polygon.setupJAPolygons();#TODO make proper
}

var lsnr = setlistener("ja37/supported/initialized", poly_start);
