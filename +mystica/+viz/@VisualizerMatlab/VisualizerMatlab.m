classdef VisualizerMatlab < handle
    %VISUALIZERMATLAB Summary of this class goes here
    %   Detailed explanation goes here

    properties (SetAccess=protected,GetAccess=public)
        stgsVisualizer
    end
    properties (SetAccess=protected,GetAccess=protected)
        figure
        figureFrames struct
        data
        model
        indexIterationVis
        indexesVisualizer
        indexesStatusTrackerPrint
        figureMatrixLimits
        structLink
        structDesiredShape
        structJoint
        structJointShared
        structPlotStatistics
        stgsIntegrator
        stgsDesiredShape
        functionDesiredNormal
        dataLiveStatistics
    end
    methods
        function obj = VisualizerMatlab(input)
            %VISUALIZERMATLAB Construct an instance of this class
            %   Detailed explanation goes here
            arguments
                input.data
                input.stgs
                input.model
                input.functionDesiredNormal
                input.dataLiveStatistics = {}
            end
            obj.data                  = input.data;
            obj.model                 = input.model;
            obj.stgsVisualizer        = input.stgs.visualizer;
            obj.stgsIntegrator        = input.stgs.integrator;
            obj.stgsDesiredShape      = input.stgs.desiredShape;
            obj.getFunctionDesiredNormal();
            obj.figure                = figure;
            obj.dataLiveStatistics    = input.dataLiveStatistics;

            obj.stgsVisualizer.joint.cone.stlName   = 'mystica_cone45.stl';
            obj.stgsVisualizer.joint.sphere.stlName = 'mystica_sphere.stl';

            obj.getIndexesSimulation('stgsIntegrator',obj.stgsIntegrator)
            obj.getFigureMatrixLimits('MatrixMBodyPos_0',input.data.mBodyPosQuat_0(input.model.selector.indexes_mBodyPos_from_mBodyPosQuat,:))

            set(obj.figure,'Units','pixels');
            set(obj.figure,'Position',obj.stgsVisualizer.figure.position);
            set(obj.figure,'WindowState',obj.stgsVisualizer.figure.windowState);
            set(obj.figure,'color',obj.stgsVisualizer.figure.backgroundColor);
            set(gca,'Units','normalized','Position',[0,0,1,1]);

            obj.createPlot(input.stgs.unitMeas)

            xlim(xlim)
            ylim(ylim)
            ZlimTemp = zlim;
            %zlim([ min([ZlimTemp(1) obj.figureMatrixLimits(3,1)]) , max([ZlimTemp(2) obj.figureMatrixLimits(3,2)]) ])
            %zlim([obj.figureMatrixLimits(3,1) obj.figureMatrixLimits(3,2)])

            view(obj.stgsVisualizer.cameraView.mBodySimulation.values)
            camlight('headlight');
            material('shiny');
        end

        function clearProperties(obj)
            obj.figure       = [];
            obj.figureFrames = [];
        end

    end
    methods (Access=protected)
        getIndexesSimulation(obj,input)
        getFigureMatrixLimits(obj,input)
        plotLiveStatistics(obj,input)
        createPlot(obj,stgsUM)
        updatePlot(obj,k)
        vertices = getConeVertices(obj,mBodyPosQuat_0,i)
        vertices = getSphereVertices(obj,mBodyPosQuat_0,i)
        color    = getSphereColor(obj,j)
        rotateCameraView(obj,durationTotal,durationInitialPhase,durationFinalPhase,initialView,finalView,counterFrames)

        function getFunctionDesiredNormal(obj)
            if obj.stgsDesiredShape.invertNormals
                signGrad = -1;
            else
                signGrad = 1;
            end
            pos_0_b   = casadi.SX.sym('pos_0_b',3,1);
            t         = casadi.SX.sym('t',1);
            xLink     = pos_0_b(1);
            yLink     = pos_0_b(2);
            funExp    = obj.stgsDesiredShape.fun(xLink,yLink,t);
            funImp    = funExp - pos_0_b(3);
            grad      = signGrad * gradient(funImp,pos_0_b);
            normalDes = grad/norm(grad);
            obj.functionDesiredNormal = casadi.Function('normalDes',{pos_0_b,t},{normalDes});
        end

    end
    methods (Static)
        varargout = plotCSYS(H,scaleLengthCSYS,prepareFigure,structPlot);
        vertices  = getCoordinatesVerticesSTL(input);
    end
end
