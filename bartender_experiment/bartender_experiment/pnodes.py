import math

from cognitive_nodes.pnode import PNode
from cognitive_nodes.perception import perception_dict_to_msg
from perception_emdb_test.perceptions import ObjectClass, EmotionClass
from core.utils import separate_perceptions


class PNodeBartenderClient(PNode):
    """
    PNode that represents a bartender client
    """
    def __init__(self, name='bartender_client', class_name='cognitive_nodes.pnode.PNode', space_class=None, space=None, history_size=100, **params):
        super().__init__(name, class_name, space_class, space, history_size, **params)

    def calculate_activation(self, perception=None, activation_list=None):
        """
        Calculate the new activation value for a given perception

        :param perception: The perception for which PNode activation is calculated.
        :type perception: dict
        :return: If there is space, returns the activation of the PNode. If not, returns 0
        :rtype: float
        """
        if activation_list!=None:
            perception={}
            for sensor in activation_list:
                activation_list[sensor]['updated']=False
                perception[sensor]=activation_list[sensor]['data']

        if perception:
            activations = []
            perceptions = next(iter(perception.values()), None)
            self.get_logger().debug(f'PNODE DEBUG: Perception: {perceptions}')
            empty_face = any([math.isnan(value) for value in perceptions[0].values()])
            if not empty_face:
                for face in perceptions:
                    emotion = EmotionClass.decode(face.get("emotion", 1.0))
                    if emotion == "SADNESS":
                        activation_value = 1.0
                        self.get_logger().debug(f'PNODE DEBUG: Perception: {perceptions} Activation: {activation_value}')
                    else:
                        activation_value = 0.0

                    activations.append(activation_value)
            else:
                activations.append(0.0) 
            self.activation.activation = activations[0] if len(activations) == 1 else float(max(activations)) #Fix this else case for multiple perceptions
            self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation
    

class PNodeHumanPresent(PNode):
    """
    PNode that represents that a human is present
    """
    def __init__(self, name='human_present', class_name='cognitive_nodes.pnode.PNode', space_class=None, space=None, history_size=100, invert=False, **params):
        super().__init__(name, class_name, space_class, space, history_size, **params)
        self.invert = invert


    def calculate_activation(self, perception=None, activation_list=None):
        """
        Calculate the new activation value for a given perception

        :param perception: The perception for which PNode activation is calculated.
        :type perception: dict
        :return: If there is space, returns the activation of the PNode. If not, returns 0
        :rtype: float
        """
        if activation_list!=None:
            perception={}
            for sensor in activation_list:
                activation_list[sensor]['updated']=False
                perception[sensor]=activation_list[sensor]['data']

        if perception:
            activations = []
            perceptions = next(iter(perception.values()), None)
            empty_face = any([math.isnan(value) for value in perceptions[0].values()])
            if not empty_face:
                self.activation.activation = 0.95 if not self.invert else 0.0
            else:
                self.activation.activation = 0.0 if not self.invert else 1.0
            self.get_logger().debug(f'PNODE DEBUG: Perception: {perceptions} Activation: {self.activation.activation}')
            self.activation.timestamp = self.get_clock().now().to_msg()
        return self.activation